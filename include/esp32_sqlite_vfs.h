#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "sqlite3.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Injectable POSIX I/O operations table for the esp32 SQLite VFS.
 * Any NULL function pointer falls back to the default POSIX implementation.
 * If max_file_offset > 0 and smaller than the platform's off_t maximum, the
 * VFS enforces max_file_offset as the representable file offset ceiling.
 */
typedef struct esp32_sqlite_io_ops {
    void *ctx;
    int (*open_fn)(void *ctx, const char *path, int oflag, mode_t mode);
    int (*close_fn)(void *ctx, int fd);
    ssize_t (*read_fn)(void *ctx, int fd, void *buf, size_t count);
    ssize_t (*write_fn)(void *ctx, int fd, const void *buf, size_t count);
    off_t (*lseek_fn)(void *ctx, int fd, off_t offset, int whence);
    int (*ftruncate_fn)(void *ctx, int fd, off_t length);
    int (*fsync_fn)(void *ctx, int fd);
    int (*fstat_fn)(void *ctx, int fd, struct stat *st);
    int (*stat_fn)(void *ctx, const char *path, struct stat *st);
    int (*unlink_fn)(void *ctx, const char *path);
    int (*dir_fsync_fn)(void *ctx, const char *file_path);
    sqlite3_int64 max_file_offset;
} esp32_sqlite_io_ops_t;

/**
 * Returns the default POSIX I/O operations table used by the "esp32" VFS.
 */
const esp32_sqlite_io_ops_t *esp32_sqlite_default_io_ops(void);

/**
 * Registers a custom VFS instance backed by the production esp32 VFS logic
 * with caller-supplied I/O hooks (e.g. for deterministic crash/fault injection).
 *
 * out_vfs_storage must remain valid until esp32_sqlite_vfs_unregister_custom()
 * is called. ops (if non-NULL) must also remain valid while registered.
 */
int esp32_sqlite_vfs_register_custom(
    const char *vfs_name,
    const esp32_sqlite_io_ops_t *ops,
    int make_default,
    sqlite3_vfs *out_vfs_storage);

/**
 * Unregisters a previously registered custom esp32 SQLite VFS instance.
 */
int esp32_sqlite_vfs_unregister_custom(sqlite3_vfs *vfs_storage);

#ifdef __cplusplus
}
#endif
