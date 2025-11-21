
/*
  Copyright (C) 2006, 2007, 2008 Peter Robinson
  Email: pjr@itee.uq.edu.au

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

//
// Code.h - For storing and executing subscriptions
//

#ifndef CODEBUFFER_H
#define CODEBUFFER_H

#include <glib.h>

#include "pheap.h"
#include "objects.h"

G_BEGIN_DECLS

#define FAIL        0
#define TTRUE       1
#define TRY         2
#define RETRY       3
#define NOTTRY      4
#define IFTRY       5
#define TRUST       6
#define ONCE        7
#define EXIT        8
#define JUMP        9
#define CUT        10
#define NOOP       11
#define UNIFY      12
#define IS         13
#define LT         14
#define GT         15
#define LE         16
#define GE         17
#define MEMBERTRY  18
#define MEMBER     19 
#define SPLITTRY   20
#define SPLIT      21 
#define SPLITSTRINGTRY   22
#define SPLITSTRING      23 
#define NUMBER     24 
#define ATOM       25 
#define STRING     26 
#define LIST       27 

typedef struct _CodeBuffer CodeBuffer;

struct _CodeBuffer
{
  guint64* codeblock;
  guint64* top;
  guint64* high;
  int maxindex;
  int bsize;

  guint64* stack;
  guint64* stacktop;
  guint64* stackhigh;
};

CodeBuffer* codebuffer_new(size_t s);

void codebuffer_destroy(CodeBuffer* cb);

#define CODE_BUFFER_RESET(cb) cb->maxindex = -1

void setCodeWord(CodeBuffer* cb, int index, guint64 w);

#define GET_CODE_WORD(cb, index) cb->codeblock[index]

void codebuffer_push(CodeBuffer* cb, guint64 ind);

    
#define CODE_BUFFER_GET_SIZE(cb) cb->maxindex+1 
    
#define CODEBLOCK_PTR(cb) cb->codeblock

void optimize(CodeBuffer* cb);

void copyCodeblock(CodeBuffer* cb, guint64* cpblock, int size);

G_END_DECLS

#endif
