
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
#ifndef VARMAP_H
#define VARMAP_H

#include <glib.h>
#include <assert.h>
#include <string.h>

#include "objects.h"
#include "pheap.h"

G_BEGIN_DECLS

void delete_var_key(gpointer str);
void delete_var(gpointer str);
gboolean clean_it(gpointer key, gpointer value, gpointer user_data);

typedef struct _VarMap VarMap;

struct _VarMap {
  GHashTable* theHashTable;
  Heap* heap;
};

VarMap* varmap_new(Heap* h);

void varmap_destroy(VarMap* vm);

ObjectPtr varmap_add(VarMap* vm, char* key);

void varmap_clean(VarMap* vm);   
    
G_END_DECLS

#endif // VARMAP_H
