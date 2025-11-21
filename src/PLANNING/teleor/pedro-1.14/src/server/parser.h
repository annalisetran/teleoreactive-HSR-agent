
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
#ifndef PARSER_H
#define PARSER_H

#include <glib.h>
#include <unistd.h>
#include "objects.h"
#include "pheap.h"
#include "atomTable.h"
#include "varMap.h"
#include "pstack.h"

G_BEGIN_DECLS

ObjectPtr 
parseIt(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);

ObjectPtr parse(char* buff, size_t size, Heap* heap, AtomTable* atoms, 
		VarMap* vmap, Stack* stk);

G_END_DECLS

#endif // PARSER_H
