/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <reent.h>
#include <unistd.h>
#include <sys/wait.h>

// #include "ff.h"

#include "coleco_rom_blob.h"
#include "zaxxon_col_blob.h"
#include "zaxxon_txt_blob.h"

#undef errno
extern int errno;

// #define USE_FATFS


int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}
}

#define ROM_NONE 0
#define ROM_ZAXXON_TXT 1
#define ROM_ZAXXON_COL 2
#define ROM_COLECO_ROM 3

#define MAX_FILES 4
enum { FD_OFFSET = 3 };
static int filesOpened[MAX_FILES];

#ifdef USE_FATFS

static FIL files[MAX_FILES];    /* starting with fd=3, so fd 3 through 3 + MAX_FILES - 1 */

#else 

static int romFile[MAX_FILES];
static int romSeek[MAX_FILES];

#endif /* USE_FATFS */

int putchar_raw(int);
int getchar_timeout_us(uint32_t timeout_us);

int _write(int file, char *ptr, int len)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	int DataIdx;

		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
		   putchar_raw(*ptr++); // __io_putchar( *ptr++ );
		}
	return len;
    } else {
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX write: file not opened\n");
            errno = EBADF;
            return -1;
        }
#ifdef USE_FATFS
        unsigned int wrote;
        FRESULT result = f_write(&files[myFile], ptr, len, &wrote);
        if(result != FR_OK) {
            printf("XXX write: file result %d\n", result);
            errno = EIO;
            return -1;
        }
        return wrote;
#else /* not USE_FATFS */
        errno = EIO;
        return -1;
#endif /* USE_FATFS */
    }
}

int _close(int file)
{
    int myFile = file - FD_OFFSET;
    if(!filesOpened[myFile]) {
        errno = EBADF;
        return -1;
    }
#ifdef USE_FATFS
    f_close(&files[myFile]);
#else
    romFile[myFile] = ROM_NONE;
#endif /* USE_FATFS */
    filesOpened[myFile] = 0;
    return 0;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	return 0;
    } else {
#ifdef USE_FATFS
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX lseek: file not opened %d\n", myFile);
            errno = EBADF;
            return -1;
        }
        FRESULT result;
        if(dir == SEEK_SET) {
            result = f_lseek(&files[myFile], ptr);
        } else if(dir == SEEK_CUR) {
            result = f_lseek(&files[myFile], ptr + f_tell(&files[myFile]));
        } else /* SEEK_END */ {
            result = f_lseek(&files[myFile], f_size(&files[myFile]) - 1 - ptr);
        }
        if(result != FR_OK) {
            printf("XXX lseek: result not OK %d\n", result);
            errno = EIO;
            return -1;
        }
        return f_tell(&files[myFile]);
#else /* not USE_FATFS */
        errno = EIO;
        return -1;
#endif /* USE_FATFS */
    }
}

int _read(int file, char *ptr, int len)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
	  *ptr++ = getchar_timeout_us(1000000); // __io_getchar();
	}
        return len;
    } else {
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX read: file not opened %d\n", myFile);
            errno = EBADF;
            return -1;
        }
        unsigned int wasRead;
#ifdef USE_FATFS
        FRESULT result = f_read(&files[myFile], ptr, len, &wasRead);
        if(result != FR_OK) {
            printf("XXX read: result not OK %d\n", result);
            errno = EIO;
            return -1;
        }
#else /* not USE_FATFS */
        const uint8_t *blob = NULL;
        int blob_length = 0;
        switch(romFile[myFile])
        {
            case ROM_ZAXXON_TXT:
                blob = zaxxon_txt_bytes;
                blob_length = zaxxon_txt_length;
                break;
            case ROM_ZAXXON_COL:
                blob = zaxxon_col_bytes;
                blob_length = zaxxon_col_length;
                break;
            case ROM_COLECO_ROM:
                blob = coleco_rom_bytes;
                blob_length = coleco_rom_length;
                break;
        }
        wasRead = (romSeek[myFile] + len > blob_length) ? (blob_length - romSeek[myFile]) : len;
        printf("read %d bytes from %d at %d\n", wasRead, romFile[myFile], romSeek[myFile]);
        memcpy(ptr, blob + romSeek[myFile], wasRead);
        romSeek[myFile] += wasRead;
#endif /* USE_FATFS */
        return wasRead;
        // errno = EIO;
        // return -1;
    }
}

int _open(char *path, int flags, ...)
{
    if(path == NULL) {
        errno = EFAULT;
        return -1;
    }

    int which = 0;
    while(which < MAX_FILES && filesOpened[which]) {
        which++;
    }
    if(which >= MAX_FILES) {
        errno = ENFILE;
        return -1;
    }

#ifdef USE_FATFS
    int FatFSFlags = 0;

    if((flags & O_ACCMODE) == O_RDONLY) {
        FatFSFlags |= FA_READ | FA_OPEN_EXISTING;
    } else if((flags & O_ACCMODE) == O_WRONLY) {
        FatFSFlags |= FA_WRITE;
    } else if((flags & O_ACCMODE) == O_RDWR) {
        FatFSFlags |= FA_WRITE | FA_READ;
    }

    if(flags & O_APPEND) {
        FatFSFlags |= FA_OPEN_APPEND;
    }
    if(flags & O_CREAT) {
        FatFSFlags |= FA_CREATE_NEW;
    }
    if(flags & O_TRUNC) {
        FatFSFlags |= FA_CREATE_ALWAYS;
    }
    errno = 0;
    FRESULT result = f_open (&files[which], path, FatFSFlags);
    if(result) {
        printf("XXX open couldn't open \"%s\" for reading, FatFS result %d\n", path, result);
        errno = EIO;
        return -1;
    }
    filesOpened[which] = 1;

    return which + FD_OFFSET;
#else /* not USE_FATFS */
    int found = ROM_NONE;
    if(strcmp(path, "coleco/COLECO.ROM") == 0) 
    {
        found = ROM_COLECO_ROM;
    }
    else if(strcmp(path, "zaxxon.col") == 0) 
    {
        found = ROM_ZAXXON_COL;
    }
    else if(strcmp(path, "zaxxon.txt") == 0) 
    {
        found = ROM_ZAXXON_TXT;
        romSeek[which] = 0;
    }
    if(found != ROM_NONE)
    {
        romFile[which] = found;
        romSeek[which] = 0;
        printf("opened %s as %d, file %d\n", path, found, which + FD_OFFSET);
        filesOpened[which] = 1;
        return which + FD_OFFSET;
    }
    errno = EIO;
    return -1;
#endif /* USE_FATFS */
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}
