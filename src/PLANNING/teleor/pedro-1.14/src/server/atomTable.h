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
#ifndef ATOMTABLE_H
#define  ATOMTABLE_H

#include <glib.h>
#include <assert.h>
#include <string.h>

#include "objects.h"
#include "pheap.h"

G_BEGIN_DECLS


void delete_key(gpointer str);
void delete_atom(gpointer str);

typedef struct _AtomTable AtomTable;

AtomTable* atom_table_new();

ObjectPtr add_atom(AtomTable* at, char* key);

ObjectPtr add_any_atom(AtomTable* at, Heap* heap, gboolean dynamic, char* key);

void atom_table_destroy(AtomTable* at);

void initAtomRef(AtomTable* at);
void pushAtomRef(AtomTable* at, ObjectPtr atomptr);
void dec_all_atom_ref(AtomTable* at);

void dec_atom_refcount(AtomTable* at, ObjectPtr atomptr);
void inc_atom_refcount(ObjectPtr gobjp);
void dec_refcount_all_atoms(AtomTable* at, ObjectPtr objptr);

G_END_DECLS

#endif //  STRINGTABLE_H
