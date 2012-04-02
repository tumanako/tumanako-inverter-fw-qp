
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
//   Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>
//   Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
//
//   This file is part of Tumanako_QP.
//
//   Tumanako_QP is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   Tumanako_QP is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with Tumanako_QP.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------


// This glue implements OS level hooks required for alocating memory and other good things

#include "bsp.h"

extern "C" {
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>                   // for prototypes of malloc() and free()

  extern char *  __heap_start__;	//check libopenstm32.ld
  extern char *  __heap_end__;

  //TODO: PCC - Not sure if these are safely implemented?
  void __exidx_start(int) {};
  void __exidx_end(int) {};
  //void exit(int a) {while (1){}};
  void abort() {
    while (1) {}};

  // Don't use dynamic memory allocation at all.
  //............................................................................
  void *malloc(size_t) {
      return (void *)0;
  }
  //............................................................................
  void free(void *) {
  }

  // Increase program data space (minimal implementation).
  // As malloc and related functions depend on this, it is useful (in our case essential) to have a working implementation.
  // The following suffices for a standalone system;
  // it exploits the symbols '__heap_start__' and '__heap_end__'  defined by the GNU linker script.

  caddr_t _sbrk(int incr) {
	  return 0;
  }

  void _init(void) {}	// TODO What should this function do?

  //Close a file (minimal implementation).
  int _close(int) {
    return -1;
  };

  // Added this BRM
  int _open(int) {
    return -1;
  };

  //Set position in a file (minimal implementation).
  //parameters: int file, int ptr, int dir
  int _lseek(int, int, int) {
    return 0;
  };

  //Read from a file (minimal implementation)
  //parameters: int file, char *ptr, int len
  int _read(int, char, int) {
    return 0;
  };

  int _write(int fd, const u8 *buf, int len)
  {
	  return 0;
  }

  //Status of an open file (minimal implementation).
  //parameters: int file, struct stat *st
  int _fstat(int, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  //Query whether output stream is a terminal (minimal implementation).
  //prameters: int file
  int _isatty(int) {
    return 1;
  }

  // DOn't need this code in an embedded system
  void __cxa_atexit(void (*arg1)(void*), void* arg2, void* arg3)
  {
  }
  void*   __dso_handle = (void*) &__dso_handle;


  int __aeabi_atexit(void *object, void (*destructor)(void *), void *dso_handle)
  {
      return 0;
  }

}

// Versions of new and delete that don't pull in C++ exception handling code.
// This should save some 50k of code space..
//............................................................................
void *operator new(size_t size) throw() {
    return malloc(size);
}
//............................................................................
void operator delete(void *p) throw() {
    free(p);
}

void __cxa_guard_acquire() {
}
//............................................................................
void __cxa_guard_release() {
}







