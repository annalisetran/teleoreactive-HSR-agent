
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
#include <stdio.h>
#include <stdlib.h>

#include "ptrail.h"
#include "atomTable.h"
#include "codeBuffer.h"

extern void logit(char* fmt, ...);

/*
 * The code buffer is used as a working area in which to build compiled code
 */

CodeBuffer* codebuffer_new(size_t s)
{
  CodeBuffer* buff =  g_new(CodeBuffer, 1);
  buff->codeblock = g_new(guint64, s);
  buff->top = buff->codeblock;
  buff->bsize = s;
  buff->high = buff->codeblock + s;
  buff->maxindex = -1;
  buff->stack = g_new(guint64, s);
  buff->stacktop = buff->stack;
  buff->stackhigh = buff->stack + s;
  return buff; 
}

void codebuffer_destroy(CodeBuffer* cb)
{
  g_free(cb->codeblock);
  g_free(cb->stack);
  g_free(cb);
}

void setCodeWord(CodeBuffer* cb, int index, guint64 w)
{
  assert(index >= 0);
  assert(index < cb->bsize);
  cb->codeblock[index] = w;
  if (index > cb->maxindex) cb->maxindex = index;
}
		 
void codebuffer_push(CodeBuffer* cb, guint64 ind)
{
  if (cb->stacktop + 1 >= cb->stackhigh)
    {
      logit("Out of code stack space\n");
      exit(1);
    }
  *(cb->stacktop) = ind;
  cb->stacktop++;
}

guint64 codebuffer_pop(CodeBuffer* cb)
{
  assert(cb->stacktop >= cb->stack);
  cb->stacktop--;
  return *(cb->stacktop);
}

    
/*
 * optimize removes shortens chains of jumps where possible
 */
void optimize(CodeBuffer* cb)
{
  int i = 0;
  while (i <= cb->maxindex)
    {
      switch(cb->codeblock[i]) {
      case TTRUE :
      case FAIL :
	i++;
	break;
      case TRY:
      case RETRY:
      case NOTTRY:
      case IFTRY:
	i += 2;
	break;
      case TRUST :
	i++;
	break;
      case ONCE :
	i++;
	break;
      case EXIT :
	i++;
	break;
      case JUMP:
	{
	  i++;
	  int jmp = cb->codeblock[i];
	  int op = cb->codeblock[jmp];
	  if (op == JUMP)
	    {
	      cb->codeblock[i] = cb->codeblock[jmp+1];
	      i--;
	    }
	  else if (op == EXIT)
	    {
	      cb->codeblock[i-1] = op;
	      cb->codeblock[i] = NOOP;
	      i++;
	    }
	  else if (op == FAIL)
	    {
	      cb->codeblock[i-1] = op;
	      cb->codeblock[i] = NOOP;
	      i++;
	    }
	  
	  else if ((op == CUT) && 
		   ((cb->codeblock[jmp+1] == EXIT) || 
		    (cb->codeblock[jmp+1] == FAIL)))
	    {
	      cb->codeblock[i-1] = op;
	      cb->codeblock[i] = cb->codeblock[jmp+1];
	      i++;
	    }
	  else
	    i += 1;
	  break;
	}
      case CUT :
	i++;
	break;
      case NOOP :
	i++;
	break;
	
      case UNIFY :
      case IS :
      case LT :
      case GT :
      case LE :
      case GE :
	i += 3;
	break;
      case NUMBER:
      case ATOM:
      case STRING:
      case LIST:
	i += 2;
	break;
      case MEMBERTRY :
	i += 5;
	break;
      case SPLITTRY :
	i += 8;
	break;
      case SPLITSTRINGTRY :
	i += 7;
	break;
      default:
	assert(0);
      }
      
    }
}

/*
 * copy the code from the code buffer into the code for the subscription
 */
void copyCodeblock(CodeBuffer* cb, guint64* cpblock, int size)
{
  guint64* block = cb->codeblock;
  guint64* ptr = cpblock;


  int i = 0;
  while (i < size)
    {
      switch(block[i])
	{
	case FAIL :
	case TTRUE :
	case TRUST :
	case ONCE :
	case EXIT :
	case CUT :
	case NOOP :
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;
	case TRY:
	case RETRY:
	case NOTTRY:
	case IFTRY:
	case JUMP:
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = (guint64)(cpblock + block[i]);
	  ptr++;
	  i++;
	  break;

	  
	case UNIFY :
	case IS :
	case LT :
	case GT :
	case LE :
	case GE :
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;
	case MEMBERTRY :
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;
	case SPLITTRY :	  
          *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;
	case SPLITSTRINGTRY :
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;

	case NUMBER:
	case ATOM:
	case STRING:
	case LIST:
	  *ptr = block[i];
	  ptr++;
	  i++;
	  *ptr = block[i];
	  ptr++;
	  i++;
	  break;
	default:
	  assert(0);
	}
      
    }

}
