#include "esp32_sqlite_vfs.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifdef ESP_PLATFORM
#include <esp_random.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <spi_flash_mmap.h>
#else
#include <pthread.h>
#endif

#define ESP32_SQLITE_MAX_PATH 256
#define ESP32_SQLITE_MAX_LOCK_PATHS 12

#ifdef ESP_PLATFORM
#define ESP32_SQLITE_SECTOR_SIZE SPI_FLASH_SEC_SIZE
#else
#define ESP32_SQLITE_SECTOR_SIZE 4096
#endif

typedef struct esp32_file esp32_file_t;

typedef struct {
    bool in_use;
    char key[ESP32_SQLITE_MAX_PATH];
    uint32_t ref_count;
    uint32_t shared_count;
    uint32_t write_generation;
    esp32_file_t *reserved_owner;
    esp32_file_t *pending_owner;
    esp32_file_t *exclusive_owner;
} esp32_lock_entry_t;

struct esp32_file {
    sqlite3_file base;
    sqlite3_vfs *vfs;
    const esp32_sqlite_io_ops_t *ops;
    int fd;
    int reopen_oflag;
    int lock_level;
    uint32_t seen_generation;
    bool did_write;
    bool delete_on_close;
    bool needs_dir_sync;
    esp32_lock_entry_t *lock_entry;
    char path[ESP32_SQLITE_MAX_PATH];
};

static esp32_lock_entry_t s_lock_table[ESP32_SQLITE_MAX_LOCK_PATHS];
static int s_last_errno = 0;
static uint32_t s_temp_counter = 0U;

#ifdef ESP_PLATFORM
static portMUX_TYPE s_init_spinlock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_vfs_mutex_buf;
static SemaphoreHandle_t s_vfs_mutex = NULL;

static SemaphoreHandle_t vfs_mutex_get(void)
{
    portENTER_CRITICAL(&s_init_spinlock);
    if (s_vfs_mutex == NULL) {
        s_vfs_mutex = xSemaphoreCreateMutexStatic(&s_vfs_mutex_buf);
    }
    SemaphoreHandle_t handle = s_vfs_mutex;
    portEXIT_CRITICAL(&s_init_spinlock);
    return handle;
}

static void vfs_mutex_ensure_init(void)
{
    (void)vfs_mutex_get();
}

static void vfs_mutex_lock(void)
{
    SemaphoreHandle_t handle = vfs_mutex_get();
    if (handle != NULL) {
        (void)xSemaphoreTake(handle, portMAX_DELAY);
    }
}

static void vfs_mutex_unlock(void)
{
    SemaphoreHandle_t handle = vfs_mutex_get();
    if (handle != NULL) {
        (void)xSemaphoreGive(handle);
    }
}
#else
static pthread_mutex_t s_vfs_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t s_prng_state = 0U;

static void vfs_mutex_lock(void)
{
    (void)pthread_mutex_lock(&s_vfs_mutex);
}

static void vfs_mutex_unlock(void)
{
    (void)pthread_mutex_unlock(&s_vfs_mutex);
}
#endif

static void record_last_errno(int err)
{
    vfs_mutex_lock();
    s_last_errno = err;
    vfs_mutex_unlock();
}

static int get_last_errno(void)
{
    vfs_mutex_lock();
    const int err = s_last_errno;
    vfs_mutex_unlock();
    return err;
}

static bool is_benign_dir_sync_errno(int err)
{
    return err == EINVAL || err == EISDIR || err == ENOSYS || err == ENOTSUP;
}

static sqlite3_int64 vfs_max_off(const esp32_sqlite_io_ops_t *ops)
{
    const sqlite3_int64 native_max = (sizeof(off_t) >= 8U) ? INT64_MAX : (sqlite3_int64)INT32_MAX;
    if (ops != NULL && ops->max_file_offset > 0 && ops->max_file_offset < native_max) {
        return ops->max_file_offset;
    }
    return native_max;
}

static char fold_ascii_lower(char c, bool fold_case)
{
    if (fold_case && c >= 'A' && c <= 'Z') {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static bool canonicalize_path(const char *path, char *out, size_t out_size, bool fold_case)
{
    if (path == NULL || path[0] == '\0' || out == NULL || out_size < 2U) {
        return false;
    }

    const bool is_abs = (path[0] == '/');
    size_t out_len = 0U;
    if (is_abs) {
        out[out_len++] = '/';
        out[out_len] = '\0';
    } else {
        out[0] = '\0';
    }

    const char *p = path;
    while (*p != '\0') {
        while (*p == '/') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }

        const char *seg_start = p;
        while (*p != '\0' && *p != '/') {
            ++p;
        }
        const size_t seg_len = (size_t)(p - seg_start);

        if (seg_len == 1U && seg_start[0] == '.') {
            continue;
        }

        if (seg_len == 2U && seg_start[0] == '.' && seg_start[1] == '.') {
            if (is_abs) {
                if (out_len > 1U) {
                    while (out_len > 1U && out[out_len - 1U] != '/') {
                        --out_len;
                    }
                    if (out_len > 1U && out[out_len - 1U] == '/') {
                        --out_len;
                    }
                    out[out_len] = '\0';
                }
            } else {
                /* Relative path: pop last non-".." segment if any, else preserve "..". */
                bool can_pop = false;
                if (out_len > 0U) {
                    size_t last_start = out_len;
                    while (last_start > 0U && out[last_start - 1U] != '/') {
                        --last_start;
                    }
                    const size_t last_len = out_len - last_start;
                    if (!(last_len == 2U && out[last_start] == '.' && out[last_start + 1U] == '.')) {
                        can_pop = true;
                        out_len = (last_start > 0U) ? (last_start - 1U) : 0U;
                        out[out_len] = '\0';
                    }
                }
                if (!can_pop) {
                    const size_t need_slash = (out_len > 0U) ? 1U : 0U;
                    if (out_len + need_slash + 2U >= out_size) {
                        return false;
                    }
                    if (need_slash != 0U) {
                        out[out_len++] = '/';
                    }
                    out[out_len++] = '.';
                    out[out_len++] = '.';
                    out[out_len] = '\0';
                }
            }
            continue;
        }

        const size_t need_slash = (out_len > 0U && out[out_len - 1U] != '/') ? 1U : 0U;
        if (out_len + need_slash + seg_len >= out_size) {
            return false;
        }
        if (need_slash != 0U) {
            out[out_len++] = '/';
        }
        for (size_t i = 0U; i < seg_len; ++i) {
            out[out_len++] = fold_ascii_lower(seg_start[i], fold_case);
        }
        out[out_len] = '\0';
    }

    if (!is_abs && out_len == 0U) {
        out[0] = '.';
        out[1] = '\0';
    }
    return true;
}

static int default_open_fn(void *ctx, const char *path, int oflag, mode_t mode)
{
    (void)ctx;
    return open(path, oflag, mode);
}

static int default_close_fn(void *ctx, int fd)
{
    (void)ctx;
    return close(fd);
}

static ssize_t default_read_fn(void *ctx, int fd, void *buf, size_t count)
{
    (void)ctx;
    return read(fd, buf, count);
}

static ssize_t default_write_fn(void *ctx, int fd, const void *buf, size_t count)
{
    (void)ctx;
    return write(fd, buf, count);
}

static off_t default_lseek_fn(void *ctx, int fd, off_t offset, int whence)
{
    (void)ctx;
    return lseek(fd, offset, whence);
}

static int default_ftruncate_fn(void *ctx, int fd, off_t length)
{
    (void)ctx;
    return ftruncate(fd, length);
}

static int default_fsync_fn(void *ctx, int fd)
{
    (void)ctx;
    return fsync(fd);
}

static int default_fstat_fn(void *ctx, int fd, struct stat *st)
{
    (void)ctx;
    return fstat(fd, st);
}

static int default_stat_fn(void *ctx, const char *path, struct stat *st)
{
    (void)ctx;
    return stat(path, st);
}

static int default_unlink_fn(void *ctx, const char *path)
{
    (void)ctx;
    return unlink(path);
}

static int extract_parent_dir(const char *file_path, char *dir_path, size_t dir_path_size)
{
    if (file_path == NULL || file_path[0] == '\0' || dir_path == NULL || dir_path_size < 2U) {
        errno = EINVAL;
        return -1;
    }

    const char *last_slash = strrchr(file_path, '/');
    if (last_slash == NULL) {
        dir_path[0] = '.';
        dir_path[1] = '\0';
        return 0;
    }
    if (last_slash == file_path) {
        dir_path[0] = '/';
        dir_path[1] = '\0';
        return 0;
    }

    const size_t dir_len = (size_t)(last_slash - file_path);
    if (dir_len >= dir_path_size) {
        errno = ENAMETOOLONG;
        return -1;
    }
    memcpy(dir_path, file_path, dir_len);
    dir_path[dir_len] = '\0';
    return 0;
}

static int default_dir_fsync_fn(void *ctx, const char *file_path)
{
    (void)ctx;
    char dir_path[ESP32_SQLITE_MAX_PATH];
    if (extract_parent_dir(file_path, dir_path, sizeof(dir_path)) != 0) {
        return -1;
    }

    int dir_fd = open(dir_path, O_RDONLY);
    if (dir_fd < 0) {
        const int open_err = errno;
        if (open_err == ENOENT) {
            struct stat st;
            memset(&st, 0, sizeof(st));
            if (stat(dir_path, &st) == 0 && S_ISDIR(st.st_mode)) {
                errno = EISDIR;
                return -1;
            }
            if (errno == 0) {
                errno = open_err;
            }
            return -1;
        }
        errno = open_err;
        return -1;
    }

    int sync_rc = 0;
    do {
        sync_rc = fsync(dir_fd);
    } while (sync_rc != 0 && errno == EINTR);
    const int sync_err = (sync_rc != 0) ? errno : 0;

    const int close_rc = close(dir_fd);
    const int close_err = (close_rc != 0) ? errno : 0;

    if (sync_rc != 0 && !is_benign_dir_sync_errno(sync_err)) {
        errno = sync_err;
        return -1;
    }
    if (close_rc != 0) {
        errno = close_err;
        return -1;
    }
    return 0;
}

static const esp32_sqlite_io_ops_t s_default_io_ops = {
    .ctx = NULL,
    .open_fn = default_open_fn,
    .close_fn = default_close_fn,
    .read_fn = default_read_fn,
    .write_fn = default_write_fn,
    .lseek_fn = default_lseek_fn,
    .ftruncate_fn = default_ftruncate_fn,
    .fsync_fn = default_fsync_fn,
    .fstat_fn = default_fstat_fn,
    .stat_fn = default_stat_fn,
    .unlink_fn = default_unlink_fn,
    .dir_fsync_fn = default_dir_fsync_fn,
    .max_file_offset = 0,
};

const esp32_sqlite_io_ops_t *esp32_sqlite_default_io_ops(void)
{
    return &s_default_io_ops;
}

static const esp32_sqlite_io_ops_t *vfs_get_io_ops(sqlite3_vfs *vfs)
{
    if (vfs != NULL && vfs->pAppData != NULL) {
        return (const esp32_sqlite_io_ops_t *)vfs->pAppData;
    }
    return &s_default_io_ops;
}

static int io_open(const esp32_sqlite_io_ops_t *ops, const char *path, int oflag, mode_t mode)
{
    return (ops->open_fn != NULL) ? ops->open_fn(ops->ctx, path, oflag, mode)
                                  : default_open_fn(NULL, path, oflag, mode);
}

static int io_close(const esp32_sqlite_io_ops_t *ops, int fd)
{
    return (ops->close_fn != NULL) ? ops->close_fn(ops->ctx, fd)
                                   : default_close_fn(NULL, fd);
}

static ssize_t io_read(const esp32_sqlite_io_ops_t *ops, int fd, void *buf, size_t count)
{
    return (ops->read_fn != NULL) ? ops->read_fn(ops->ctx, fd, buf, count)
                                  : default_read_fn(NULL, fd, buf, count);
}

static ssize_t io_write(const esp32_sqlite_io_ops_t *ops, int fd, const void *buf, size_t count)
{
    return (ops->write_fn != NULL) ? ops->write_fn(ops->ctx, fd, buf, count)
                                   : default_write_fn(NULL, fd, buf, count);
}

static off_t io_lseek(const esp32_sqlite_io_ops_t *ops, int fd, off_t offset, int whence)
{
    return (ops->lseek_fn != NULL) ? ops->lseek_fn(ops->ctx, fd, offset, whence)
                                   : default_lseek_fn(NULL, fd, offset, whence);
}

static int io_ftruncate(const esp32_sqlite_io_ops_t *ops, int fd, off_t length)
{
    return (ops->ftruncate_fn != NULL) ? ops->ftruncate_fn(ops->ctx, fd, length)
                                       : default_ftruncate_fn(NULL, fd, length);
}

static int io_fsync(const esp32_sqlite_io_ops_t *ops, int fd)
{
    return (ops->fsync_fn != NULL) ? ops->fsync_fn(ops->ctx, fd)
                                   : default_fsync_fn(NULL, fd);
}

static int io_fstat(const esp32_sqlite_io_ops_t *ops, int fd, struct stat *st)
{
    return (ops->fstat_fn != NULL) ? ops->fstat_fn(ops->ctx, fd, st)
                                   : default_fstat_fn(NULL, fd, st);
}

static int io_stat(const esp32_sqlite_io_ops_t *ops, const char *path, struct stat *st)
{
    return (ops->stat_fn != NULL) ? ops->stat_fn(ops->ctx, path, st)
                                  : default_stat_fn(NULL, path, st);
}

static int io_unlink(const esp32_sqlite_io_ops_t *ops, const char *path)
{
    return (ops->unlink_fn != NULL) ? ops->unlink_fn(ops->ctx, path)
                                    : default_unlink_fn(NULL, path);
}

static int sync_parent_dir(const esp32_sqlite_io_ops_t *ops, const char *file_path)
{
    if (ops->dir_fsync_fn != NULL && ops->dir_fsync_fn != default_dir_fsync_fn) {
        errno = 0;
        const int rc = ops->dir_fsync_fn(ops->ctx, file_path);
        if (rc == 0) {
            return SQLITE_OK;
        }
        const int err = errno;
        if (is_benign_dir_sync_errno(err)) {
            return SQLITE_OK;
        }
        record_last_errno(err);
        return SQLITE_IOERR_DIR_FSYNC;
    }

    char dir_path[ESP32_SQLITE_MAX_PATH];
    if (extract_parent_dir(file_path, dir_path, sizeof(dir_path)) != 0) {
        record_last_errno(errno);
        return SQLITE_IOERR_DIR_FSYNC;
    }

    errno = 0;
    const int dir_fd = io_open(ops, dir_path, O_RDONLY, 0);
    if (dir_fd < 0) {
        const int open_err = errno;
        if (open_err == ENOENT) {
            /* ESP-IDF FATFS f_open() returns FR_NO_FILE (mapped to ENOENT) on directories.
             * Distinguish an existing directory from a genuinely missing directory or media error. */
            struct stat st;
            memset(&st, 0, sizeof(st));
            errno = 0;
            if (io_stat(ops, dir_path, &st) == 0 && S_ISDIR(st.st_mode)) {
                return SQLITE_OK;
            }
            record_last_errno(errno != 0 ? errno : open_err);
            return SQLITE_IOERR_DIR_FSYNC;
        }
        if (is_benign_dir_sync_errno(open_err)) {
            return SQLITE_OK;
        }
        record_last_errno(open_err);
        return SQLITE_IOERR_DIR_FSYNC;
    }

    int sync_rc = 0;
    do {
        errno = 0;
        sync_rc = io_fsync(ops, dir_fd);
    } while (sync_rc != 0 && errno == EINTR);
    const int sync_err = (sync_rc != 0) ? errno : 0;

    errno = 0;
    const int close_rc = io_close(ops, dir_fd);
    const int close_err = (close_rc != 0) ? errno : 0;

    if (sync_rc != 0 && !is_benign_dir_sync_errno(sync_err)) {
        record_last_errno(sync_err);
        return SQLITE_IOERR_DIR_FSYNC;
    }
    if (close_rc != 0) {
        record_last_errno(close_err);
        return SQLITE_IOERR_DIR_FSYNC;
    }
    return SQLITE_OK;
}

static esp32_lock_entry_t *lock_entry_acquire(const char *path, uint32_t *out_generation)
{
    char key[ESP32_SQLITE_MAX_PATH];
#ifdef ESP_PLATFORM
    const bool fold_case = true;
#else
    const bool fold_case = false;
#endif
    if (!canonicalize_path(path, key, sizeof(key), fold_case)) {
        return NULL;
    }

    vfs_mutex_lock();
    esp32_lock_entry_t *free_slot = NULL;
    for (size_t i = 0U; i < ESP32_SQLITE_MAX_LOCK_PATHS; ++i) {
        if (s_lock_table[i].in_use) {
            if (strcmp(s_lock_table[i].key, key) == 0) {
                s_lock_table[i].ref_count++;
                if (out_generation != NULL) {
                    *out_generation = s_lock_table[i].write_generation;
                }
                vfs_mutex_unlock();
                return &s_lock_table[i];
            }
        } else if (free_slot == NULL) {
            free_slot = &s_lock_table[i];
        }
    }

    if (free_slot == NULL) {
        vfs_mutex_unlock();
        return NULL;
    }

    memset(free_slot, 0, sizeof(*free_slot));
    free_slot->in_use = true;
    free_slot->ref_count = 1U;
    (void)snprintf(free_slot->key, sizeof(free_slot->key), "%s", key);
    if (out_generation != NULL) {
        *out_generation = free_slot->write_generation;
    }
    vfs_mutex_unlock();
    return free_slot;
}

static void lock_entry_unlock_locked(esp32_file_t *file, int target_level)
{
    esp32_lock_entry_t *entry = file->lock_entry;
    if (entry == NULL || file->lock_level <= target_level) {
        return;
    }

    const bool had_write_lock = (file->lock_level >= SQLITE_LOCK_RESERVED);
    if (target_level < SQLITE_LOCK_EXCLUSIVE && entry->exclusive_owner == file) {
        entry->exclusive_owner = NULL;
    }
    if (target_level < SQLITE_LOCK_PENDING && entry->pending_owner == file) {
        entry->pending_owner = NULL;
    }
    if (target_level < SQLITE_LOCK_RESERVED && entry->reserved_owner == file) {
        entry->reserved_owner = NULL;
    }
    if (target_level < SQLITE_LOCK_RESERVED && (file->did_write || had_write_lock)) {
        if (file->did_write) {
            entry->write_generation++;
            file->seen_generation = entry->write_generation;
            file->did_write = false;
        }
    }
    if (target_level == SQLITE_LOCK_NONE && file->lock_level >= SQLITE_LOCK_SHARED) {
        if (entry->shared_count > 0U) {
            entry->shared_count--;
        }
    }

    file->lock_level = target_level;
}

static void lock_entry_release(esp32_file_t *file)
{
    vfs_mutex_lock();
    esp32_lock_entry_t *entry = file->lock_entry;
    if (entry != NULL) {
        lock_entry_unlock_locked(file, SQLITE_LOCK_NONE);
        if (file->did_write) {
            entry->write_generation++;
            file->did_write = false;
        }
        if (entry->ref_count > 0U) {
            entry->ref_count--;
        }
        if (entry->ref_count == 0U) {
            memset(entry, 0, sizeof(*entry));
        }
        file->lock_entry = NULL;
    }
    vfs_mutex_unlock();
}

static int esp32_close(sqlite3_file *id)
{
    esp32_file_t *file = (esp32_file_t *)id;
    int rc = SQLITE_OK;

    lock_entry_release(file);

    if (file->fd >= 0) {
        if (io_close(file->ops, file->fd) != 0) {
            record_last_errno(errno);
            rc = SQLITE_IOERR_CLOSE;
        }
        file->fd = -1;
    }

    if (file->delete_on_close && file->path[0] != '\0') {
        if (io_unlink(file->ops, file->path) != 0 && errno != ENOENT) {
            record_last_errno(errno);
            if (rc == SQLITE_OK) {
                rc = SQLITE_IOERR_DELETE;
            }
        }
    }

    return rc;
}

static int esp32_read(sqlite3_file *id, void *buffer, int amount, sqlite3_int64 offset)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (file->fd < 0 || amount < 0 || offset < 0 || (amount > 0 && buffer == NULL)) {
        return SQLITE_IOERR_READ;
    }
    if (amount == 0) {
        return SQLITE_OK;
    }

    const sqlite3_int64 max_off = vfs_max_off(file->ops);
    if (offset > max_off || offset > max_off - (sqlite3_int64)amount) {
        memset(buffer, 0, (size_t)amount);
        return SQLITE_IOERR_SHORT_READ;
    }

    /* Check current file size BEFORE lseek() so FATFS f_lseek() on a writable FD
     * never allocates clusters or extends the file when SQLite probes past EOF. */
    struct stat st;
    memset(&st, 0, sizeof(st));
    if (io_fstat(file->ops, file->fd, &st) != 0) {
        record_last_errno(errno);
        return SQLITE_IOERR_READ;
    }
    if (st.st_size < 0) {
        return SQLITE_IOERR_READ;
    }

    const sqlite3_int64 file_size = (sqlite3_int64)st.st_size;
    if (offset >= file_size) {
        memset(buffer, 0, (size_t)amount);
        return SQLITE_IOERR_SHORT_READ;
    }

    const sqlite3_int64 avail = file_size - offset;
    const size_t want = (size_t)amount;
    const size_t to_read = ((sqlite3_int64)want > avail) ? (size_t)avail : want;
    const bool expect_short = (to_read < want);

    const off_t pos = io_lseek(file->ops, file->fd, (off_t)offset, SEEK_SET);
    if (pos == (off_t)-1 || (sqlite3_int64)pos != offset) {
        record_last_errno(errno);
        return SQLITE_IOERR_READ;
    }

    uint8_t *dst = (uint8_t *)buffer;
    size_t total = 0U;

    while (total < to_read) {
        const ssize_t got = io_read(file->ops, file->fd, dst + total, to_read - total);
        if (got > 0) {
            total += (size_t)got;
        } else if (got == 0) {
            memset(dst + total, 0, want - total);
            return SQLITE_IOERR_SHORT_READ;
        } else {
            if (errno == EINTR) {
                continue;
            }
            record_last_errno(errno);
            return SQLITE_IOERR_READ;
        }
    }

    if (expect_short) {
        memset(dst + total, 0, want - total);
        return SQLITE_IOERR_SHORT_READ;
    }

    return SQLITE_OK;
}

static int esp32_write(sqlite3_file *id, const void *buffer, int amount, sqlite3_int64 offset)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (file->fd < 0 || amount < 0 || offset < 0 || (amount > 0 && buffer == NULL)) {
        return SQLITE_IOERR_WRITE;
    }
    if (amount == 0) {
        return SQLITE_OK;
    }

    const sqlite3_int64 max_off = vfs_max_off(file->ops);
    if (offset > max_off || offset > max_off - (sqlite3_int64)amount) {
        return SQLITE_FULL;
    }

    const off_t pos = io_lseek(file->ops, file->fd, (off_t)offset, SEEK_SET);
    if (pos == (off_t)-1 || (sqlite3_int64)pos != offset) {
        record_last_errno(errno);
        return SQLITE_IOERR_SEEK;
    }

    const uint8_t *src = (const uint8_t *)buffer;
    size_t total = 0U;
    const size_t want = (size_t)amount;

    while (total < want) {
        errno = 0;
        const ssize_t wrote = io_write(file->ops, file->fd, src + total, want - total);
        if (wrote > 0) {
            total += (size_t)wrote;
        } else if (wrote == 0) {
            const int err = errno;
            record_last_errno(err);
            return (err == ENOSPC) ? SQLITE_FULL : SQLITE_IOERR_WRITE;
        } else {
            if (errno == EINTR) {
                continue;
            }
            const int err = errno;
            record_last_errno(err);
            return (err == ENOSPC) ? SQLITE_FULL : SQLITE_IOERR_WRITE;
        }
    }

    vfs_mutex_lock();
    file->did_write = true;
    vfs_mutex_unlock();
    return SQLITE_OK;
}

static int esp32_truncate(sqlite3_file *id, sqlite3_int64 bytes)
{
    esp32_file_t *file = (esp32_file_t *)id;
    const sqlite3_int64 max_off = vfs_max_off(file->ops);
    if (file->fd < 0 || bytes < 0 || bytes > max_off) {
        return SQLITE_IOERR_TRUNCATE;
    }

    int rc = 0;
    do {
        rc = io_ftruncate(file->ops, file->fd, (off_t)bytes);
    } while (rc != 0 && errno == EINTR);

    if (rc != 0) {
        record_last_errno(errno);
        return SQLITE_IOERR_TRUNCATE;
    }

    vfs_mutex_lock();
    file->did_write = true;
    vfs_mutex_unlock();
    return SQLITE_OK;
}

static int esp32_sync(sqlite3_file *id, int flags)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (file->fd < 0) {
        return SQLITE_IOERR_FSYNC;
    }

    int rc = 0;
    do {
        rc = io_fsync(file->ops, file->fd);
    } while (rc != 0 && errno == EINTR);

    if (rc != 0) {
        const int err = errno;
        record_last_errno(err);
        return (err == ENOSPC) ? SQLITE_FULL : SQLITE_IOERR_FSYNC;
    }

    if (file->needs_dir_sync && (flags & SQLITE_SYNC_DATAONLY) == 0) {
        const int dir_rc = sync_parent_dir(file->ops, file->path);
        if (dir_rc != SQLITE_OK) {
            return dir_rc;
        }
        file->needs_dir_sync = false;
    }

    return SQLITE_OK;
}

static int esp32_file_size(sqlite3_file *id, sqlite3_int64 *size)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (file->fd < 0 || size == NULL) {
        return SQLITE_IOERR_FSTAT;
    }

    struct stat st;
    memset(&st, 0, sizeof(st));
    if (io_fstat(file->ops, file->fd, &st) != 0) {
        record_last_errno(errno);
        return SQLITE_IOERR_FSTAT;
    }

    *size = (sqlite3_int64)st.st_size;
    return SQLITE_OK;
}

static int esp32_lock(sqlite3_file *id, int lock_type)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (lock_type < SQLITE_LOCK_NONE || lock_type > SQLITE_LOCK_EXCLUSIVE) {
        return SQLITE_IOERR_LOCK;
    }

    vfs_mutex_lock();
    esp32_lock_entry_t *entry = file->lock_entry;
    if (entry == NULL) {
        vfs_mutex_unlock();
        return SQLITE_IOERR_LOCK;
    }

    if (file->lock_level >= lock_type) {
        vfs_mutex_unlock();
        return SQLITE_OK;
    }

    int rc = SQLITE_OK;

    if (file->lock_level == SQLITE_LOCK_NONE) {
        if ((entry->pending_owner != NULL && entry->pending_owner != file) ||
            (entry->exclusive_owner != NULL && entry->exclusive_owner != file)) {
            rc = SQLITE_BUSY;
            goto done;
        }
        entry->shared_count++;
        file->lock_level = SQLITE_LOCK_SHARED;

        /* Refresh open descriptor if another handle committed writes while this handle was
         * unlocked, avoiding stale FATFS FIL size/cluster-chain caches. */
        if (!file->delete_on_close && file->path[0] != '\0' && file->seen_generation != entry->write_generation) {
            const uint32_t target_gen = entry->write_generation;
            vfs_mutex_unlock();

            const int new_fd = io_open(file->ops, file->path, file->reopen_oflag, 0666);
            if (new_fd < 0) {
                record_last_errno(errno);
                vfs_mutex_lock();
                lock_entry_unlock_locked(file, SQLITE_LOCK_NONE);
                vfs_mutex_unlock();
                return SQLITE_IOERR_LOCK;
            }
            if (file->fd >= 0) {
                (void)io_close(file->ops, file->fd);
            }
            file->fd = new_fd;

            vfs_mutex_lock();
            file->seen_generation = target_gen;
        }

        if (lock_type == SQLITE_LOCK_SHARED) {
            goto done;
        }
    }

    if (lock_type == SQLITE_LOCK_RESERVED) {
        if ((entry->reserved_owner != NULL && entry->reserved_owner != file) ||
            (entry->pending_owner != NULL && entry->pending_owner != file) ||
            (entry->exclusive_owner != NULL && entry->exclusive_owner != file)) {
            rc = SQLITE_BUSY;
            goto done;
        }
        entry->reserved_owner = file;
        file->lock_level = SQLITE_LOCK_RESERVED;
        goto done;
    }

    if (lock_type == SQLITE_LOCK_PENDING) {
        if ((entry->reserved_owner != NULL && entry->reserved_owner != file) ||
            (entry->pending_owner != NULL && entry->pending_owner != file) ||
            (entry->exclusive_owner != NULL && entry->exclusive_owner != file)) {
            rc = SQLITE_BUSY;
            goto done;
        }
        entry->pending_owner = file;
        file->lock_level = SQLITE_LOCK_PENDING;
        goto done;
    }

    if (lock_type == SQLITE_LOCK_EXCLUSIVE) {
        if ((entry->reserved_owner != NULL && entry->reserved_owner != file) ||
            (entry->pending_owner != NULL && entry->pending_owner != file) ||
            (entry->exclusive_owner != NULL && entry->exclusive_owner != file)) {
            rc = SQLITE_BUSY;
            goto done;
        }
        entry->pending_owner = file;
        file->lock_level = SQLITE_LOCK_PENDING;

        if (entry->shared_count > 1U) {
            rc = SQLITE_BUSY;
            goto done;
        }

        entry->exclusive_owner = file;
        file->lock_level = SQLITE_LOCK_EXCLUSIVE;
    }

done:
    vfs_mutex_unlock();
    return rc;
}

static int esp32_unlock(sqlite3_file *id, int lock_type)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (lock_type != SQLITE_LOCK_NONE && lock_type != SQLITE_LOCK_SHARED) {
        return SQLITE_IOERR_UNLOCK;
    }

    vfs_mutex_lock();
    lock_entry_unlock_locked(file, lock_type);
    vfs_mutex_unlock();
    return SQLITE_OK;
}

static int esp32_check_reserved_lock(sqlite3_file *id, int *result)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (result == NULL) {
        return SQLITE_IOERR_CHECKRESERVEDLOCK;
    }

    vfs_mutex_lock();
    const esp32_lock_entry_t *entry = file->lock_entry;
    *result = (entry != NULL && (entry->reserved_owner != NULL ||
                                 entry->pending_owner != NULL ||
                                 entry->exclusive_owner != NULL))
                  ? 1
                  : 0;
    vfs_mutex_unlock();
    return SQLITE_OK;
}

static int esp32_file_control(sqlite3_file *id, int op, void *arg)
{
    esp32_file_t *file = (esp32_file_t *)id;
    if (op == SQLITE_FCNTL_LOCKSTATE) {
        if (arg == NULL) {
            return SQLITE_MISUSE;
        }
        vfs_mutex_lock();
        *(int *)arg = file->lock_level;
        vfs_mutex_unlock();
        return SQLITE_OK;
    }
    return SQLITE_NOTFOUND;
}

static int esp32_sector_size(sqlite3_file *id)
{
    (void)id;
    return ESP32_SQLITE_SECTOR_SIZE;
}

static int esp32_device_characteristics(sqlite3_file *id)
{
    (void)id;
    return 0;
}

static const sqlite3_io_methods s_esp32_io_methods = {
    .iVersion = 1,
    .xClose = esp32_close,
    .xRead = esp32_read,
    .xWrite = esp32_write,
    .xTruncate = esp32_truncate,
    .xSync = esp32_sync,
    .xFileSize = esp32_file_size,
    .xLock = esp32_lock,
    .xUnlock = esp32_unlock,
    .xCheckReservedLock = esp32_check_reserved_lock,
    .xFileControl = esp32_file_control,
    .xSectorSize = esp32_sector_size,
    .xDeviceCharacteristics = esp32_device_characteristics,
};

static int open_temp_file(const esp32_sqlite_io_ops_t *ops, char *out_path, size_t out_path_size)
{
    char base_dir[ESP32_SQLITE_MAX_PATH];
#ifdef ESP_PLATFORM
    (void)snprintf(base_dir, sizeof(base_dir), "/sdcard");
#else
    const char *env_tmp = getenv("TMPDIR");
    const char *raw_dir = (env_tmp != NULL && env_tmp[0] != '\0') ? env_tmp : "/tmp";
    if (!canonicalize_path(raw_dir, base_dir, sizeof(base_dir), false)) {
        (void)snprintf(base_dir, sizeof(base_dir), "/tmp");
    }
#endif

    const char *sep = (strcmp(base_dir, "/") == 0) ? "" : "/";
    for (unsigned attempt = 0U; attempt < 32U; ++attempt) {
        vfs_mutex_lock();
        const uint32_t seq = ++s_temp_counter;
        vfs_mutex_unlock();

        const int n = snprintf(out_path, out_path_size, "%s%sesp32_sqlite_%lu_%u.tmp",
                               base_dir, sep, (unsigned long)seq, attempt);
        if (n <= 0 || (size_t)n >= out_path_size) {
            errno = ENAMETOOLONG;
            return -1;
        }

        const int fd = io_open(ops, out_path, O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd >= 0) {
            return fd;
        }
        if (errno != EEXIST) {
            return -1;
        }
    }

    errno = EEXIST;
    return -1;
}

static int esp32_open(sqlite3_vfs *vfs, const char *path, sqlite3_file *file, int flags, int *outflags)
{
    esp32_file_t *p = (esp32_file_t *)file;
    memset(p, 0, sizeof(*p));
    p->fd = -1;
    p->vfs = vfs;
    p->ops = vfs_get_io_ops(vfs);

    int effective_flags = flags;
    int reopen_oflag = O_RDONLY;
    int fd = -1;

    if (path == NULL) {
        fd = open_temp_file(p->ops, p->path, sizeof(p->path));
        if (fd < 0) {
            record_last_errno(errno);
            return SQLITE_CANTOPEN;
        }
        reopen_oflag = O_RDWR;
        p->delete_on_close = true;
        p->needs_dir_sync = false;
    } else {
        if (!canonicalize_path(path, p->path, sizeof(p->path), false)) {
            return SQLITE_CANTOPEN;
        }

        const bool is_readwrite = ((flags & SQLITE_OPEN_READWRITE) != 0);
        const bool is_create = ((flags & SQLITE_OPEN_CREATE) != 0);
        const bool is_exclusive = ((flags & SQLITE_OPEN_EXCLUSIVE) != 0);
        const bool is_main_db = ((flags & SQLITE_OPEN_MAIN_DB) != 0);
        const bool is_journal = ((flags & (SQLITE_OPEN_MAIN_JOURNAL |
                                           SQLITE_OPEN_MASTER_JOURNAL |
                                           SQLITE_OPEN_SUBJOURNAL)) != 0);

        int oflag = is_readwrite ? O_RDWR : O_RDONLY;
        reopen_oflag = oflag;
        if (is_create) {
            oflag |= O_CREAT;
        }
        if (is_exclusive) {
            oflag |= O_EXCL;
        }

        fd = io_open(p->ops, p->path, oflag, 0666);
        const bool allow_ro_fallback =
            is_readwrite && !is_exclusive && !is_journal && (is_main_db || !is_create);
        if (fd < 0 && allow_ro_fallback && (errno == EACCES || errno == EROFS)) {
            fd = io_open(p->ops, p->path, O_RDONLY, 0);
            if (fd >= 0) {
                effective_flags = (flags & ~(SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE)) | SQLITE_OPEN_READONLY;
                reopen_oflag = O_RDONLY;
            }
        }

        if (fd < 0) {
            record_last_errno(errno);
            return SQLITE_CANTOPEN;
        }

        p->delete_on_close = ((flags & SQLITE_OPEN_DELETEONCLOSE) != 0);
        p->needs_dir_sync = ((effective_flags & SQLITE_OPEN_READONLY) == 0) && (is_create || is_journal);
    }

    uint32_t initial_gen = 0U;
    p->lock_entry = lock_entry_acquire(p->path, &initial_gen);
    if (p->lock_entry == NULL) {
        (void)io_close(p->ops, fd);
        if (p->delete_on_close && p->path[0] != '\0') {
            (void)io_unlink(p->ops, p->path);
        }
        return SQLITE_NOMEM;
    }

    p->fd = fd;
    p->reopen_oflag = reopen_oflag;
    p->seen_generation = initial_gen;
    p->lock_level = SQLITE_LOCK_NONE;
    p->base.pMethods = &s_esp32_io_methods;

    if (outflags != NULL) {
        *outflags = effective_flags;
    }
    return SQLITE_OK;
}

static int esp32_delete(sqlite3_vfs *vfs, const char *path, int sync_dir)
{
    if (path == NULL || path[0] == '\0') {
        return SQLITE_IOERR_DELETE;
    }

    char norm_path[ESP32_SQLITE_MAX_PATH];
    if (!canonicalize_path(path, norm_path, sizeof(norm_path), false)) {
        return SQLITE_IOERR_DELETE;
    }

    const esp32_sqlite_io_ops_t *ops = vfs_get_io_ops(vfs);
    if (io_unlink(ops, norm_path) != 0) {
        const int err = errno;
        record_last_errno(err);
        return (err == ENOENT) ? SQLITE_IOERR_DELETE_NOENT : SQLITE_IOERR_DELETE;
    }

    if (sync_dir != 0) {
        const int dir_rc = sync_parent_dir(ops, norm_path);
        if (dir_rc != SQLITE_OK) {
            return dir_rc;
        }
    }

    return SQLITE_OK;
}

static int esp32_access(sqlite3_vfs *vfs, const char *path, int flags, int *result)
{
    if (result == NULL) {
        return SQLITE_IOERR_ACCESS;
    }
    *result = 0;

    if (path == NULL || path[0] == '\0') {
        return SQLITE_OK;
    }

    char norm_path[ESP32_SQLITE_MAX_PATH];
    if (!canonicalize_path(path, norm_path, sizeof(norm_path), false)) {
        return SQLITE_IOERR_ACCESS;
    }

    const esp32_sqlite_io_ops_t *ops = vfs_get_io_ops(vfs);
    struct stat st;
    memset(&st, 0, sizeof(st));
    if (io_stat(ops, norm_path, &st) != 0) {
        const int err = errno;
        if (err == ENOENT || err == ENOTDIR) {
            *result = 0;
            return SQLITE_OK;
        }
        record_last_errno(err);
        return SQLITE_IOERR_ACCESS;
    }

    if (flags == SQLITE_ACCESS_READWRITE) {
        const mode_t perms = st.st_mode & (mode_t)0777;
        if (perms != 0) {
            const bool can_read = ((perms & (S_IRUSR | S_IRGRP | S_IROTH)) != 0);
            const bool can_write = ((perms & (S_IWUSR | S_IWGRP | S_IWOTH)) != 0);
            *result = (can_read && can_write) ? 1 : 0;
        } else {
            *result = 1;
        }
    } else {
        *result = 1;
    }

    return SQLITE_OK;
}

static int esp32_full_pathname(sqlite3_vfs *vfs, const char *path, int len, char *fullpath)
{
    (void)vfs;
    if (path == NULL || fullpath == NULL || len <= 0) {
        return SQLITE_CANTOPEN;
    }

    if (!canonicalize_path(path, fullpath, (size_t)len, false)) {
        return SQLITE_CANTOPEN;
    }
    return SQLITE_OK;
}

static int esp32_randomness(sqlite3_vfs *vfs, int len, char *buffer)
{
    (void)vfs;
    if (len <= 0 || buffer == NULL) {
        return 0;
    }

#ifdef ESP_PLATFORM
    esp_fill_random(buffer, (size_t)len);
#else
    vfs_mutex_lock();
    if (s_prng_state == 0U) {
        s_prng_state = ((uint64_t)time(NULL) << 32) ^ (uint64_t)(uintptr_t)buffer ^ (uint64_t)getpid();
        if (s_prng_state == 0U) {
            s_prng_state = 0x9E3779B97F4A7C15ULL;
        }
    }
    uint64_t x = s_prng_state;
    for (int i = 0; i < len; ++i) {
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        buffer[i] = (char)((x * 0x2545F4914F6CDD1DULL) >> 56);
    }
    s_prng_state = x;
    vfs_mutex_unlock();
#endif

    return len;
}

static int esp32_sleep(sqlite3_vfs *vfs, int microseconds)
{
    (void)vfs;
    if (microseconds <= 0) {
        return 0;
    }

#ifdef ESP_PLATFORM
    const int64_t start_us = esp_timer_get_time();
    const int64_t target_us = start_us + (int64_t)microseconds;
    const uint32_t tick_us = (uint32_t)(1000000UL / (unsigned long)configTICK_RATE_HZ);

    for (;;) {
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= target_us) {
            break;
        }
        const uint64_t rem_us = (uint64_t)(target_us - now_us);
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && tick_us > 0U && rem_us >= tick_us) {
            const uint64_t ticks = rem_us / tick_us;
            vTaskDelay((TickType_t)(ticks > 0U ? ticks : 1U));
        } else {
            const uint32_t chunk = (rem_us > 10000U) ? 10000U : (uint32_t)rem_us;
            esp_rom_delay_us(chunk);
        }
    }
#else
    struct timespec start_ts;
    if (clock_gettime(CLOCK_MONOTONIC, &start_ts) == 0) {
        const int64_t add_ns = (int64_t)microseconds * 1000LL;
        struct timespec deadline = start_ts;
        deadline.tv_sec += (time_t)(add_ns / 1000000000LL);
        deadline.tv_nsec += (long)(add_ns % 1000000000LL);
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec += 1;
            deadline.tv_nsec -= 1000000000L;
        }

        for (;;) {
            struct timespec cur_ts;
            if (clock_gettime(CLOCK_MONOTONIC, &cur_ts) != 0) {
                break;
            }
            const int64_t diff_ns = ((int64_t)(deadline.tv_sec - cur_ts.tv_sec) * 1000000000LL) +
                                    (int64_t)(deadline.tv_nsec - cur_ts.tv_nsec);
            if (diff_ns <= 0) {
                break;
            }
            struct timespec req;
            req.tv_sec = (time_t)(diff_ns / 1000000000LL);
            req.tv_nsec = (long)(diff_ns % 1000000000LL);
            (void)nanosleep(&req, NULL);
        }
    } else {
        (void)usleep((useconds_t)microseconds);
    }
#endif

    return microseconds;
}

static int esp32_current_time(sqlite3_vfs *vfs, double *result)
{
    (void)vfs;
    if (result == NULL) {
        return SQLITE_ERROR;
    }

    const time_t now = time(NULL);
    *result = ((double)now / 86400.0) + 2440587.5;
    return SQLITE_OK;
}

static int esp32_get_last_error(sqlite3_vfs *vfs, int n_buf, char *z_buf)
{
    (void)vfs;
    const int err = get_last_errno();
    if (z_buf != NULL && n_buf > 0) {
        if (err != 0) {
            (void)snprintf(z_buf, (size_t)n_buf, "%s", strerror(err));
        } else {
            z_buf[0] = '\0';
        }
    }
    return err;
}

static sqlite3_vfs s_esp32_vfs = {
    .iVersion = 1,
    .szOsFile = (int)sizeof(esp32_file_t),
    .mxPathname = ESP32_SQLITE_MAX_PATH,
    .pNext = NULL,
    .zName = "esp32",
    .pAppData = (void *)&s_default_io_ops,
    .xOpen = esp32_open,
    .xDelete = esp32_delete,
    .xAccess = esp32_access,
    .xFullPathname = esp32_full_pathname,
    .xDlOpen = NULL,
    .xDlError = NULL,
    .xDlSym = NULL,
    .xDlClose = NULL,
    .xRandomness = esp32_randomness,
    .xSleep = esp32_sleep,
    .xCurrentTime = esp32_current_time,
    .xGetLastError = esp32_get_last_error,
};

int esp32_sqlite_vfs_register_custom(
    const char *vfs_name,
    const esp32_sqlite_io_ops_t *ops,
    int make_default,
    sqlite3_vfs *out_vfs_storage)
{
    if (vfs_name == NULL || vfs_name[0] == '\0' || out_vfs_storage == NULL) {
        return SQLITE_MISUSE;
    }

    *out_vfs_storage = s_esp32_vfs;
    out_vfs_storage->pNext = NULL;
    out_vfs_storage->zName = vfs_name;
    out_vfs_storage->pAppData = (void *)(ops != NULL ? ops : &s_default_io_ops);

    return sqlite3_vfs_register(out_vfs_storage, make_default);
}

int esp32_sqlite_vfs_unregister_custom(sqlite3_vfs *vfs_storage)
{
    if (vfs_storage == NULL) {
        return SQLITE_MISUSE;
    }
    return sqlite3_vfs_unregister(vfs_storage);
}

int sqlite3_os_init(void)
{
#ifdef ESP_PLATFORM
    vfs_mutex_ensure_init();
#endif
    return sqlite3_vfs_register(&s_esp32_vfs, 1);
}

int sqlite3_os_end(void)
{
    return SQLITE_OK;
}
