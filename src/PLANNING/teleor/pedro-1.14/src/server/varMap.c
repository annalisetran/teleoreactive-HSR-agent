#include "varMap.h"
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

/*
 * Used to keep track of Prolog variables
 */

void delete_var_key(gpointer str)
{
  g_free(str);
}

void delete_var(gpointer atomp)
{
}

gboolean clean_it(gpointer key, gpointer value, gpointer user_data)
{ 
  return TRUE;
}


VarMap* varmap_new(Heap* h)
{
  VarMap* vm =  g_new(VarMap, 1);
  vm->theHashTable = g_hash_table_new_full(g_str_hash, g_str_equal, 
					   delete_var_key, delete_var);
  vm->heap = h;
  return vm; 
}

void varmap_destroy(VarMap* vm)
{
  g_hash_table_destroy(vm->theHashTable);	
  g_free(vm);
}

ObjectPtr varmap_add(VarMap* vm, char* key)
{
  if ((key[0] == '_') && (key[1] == '\0'))
    {
      return pushVariable(vm->heap);
    }
  gpointer gobjp = g_hash_table_lookup(vm->theHashTable, (gconstpointer)key);
  if (gobjp == NULL) {
      gobjp = (gpointer)pushVariable(vm->heap);
      size_t size = strlen(key);
      char* keycopy = g_malloc(size+1);
      strcpy(keycopy, key);
      g_hash_table_insert(vm->theHashTable, keycopy, gobjp);
    }
  return (ObjectPtr)gobjp;
}

void varmap_clean(VarMap* vm)
{
  g_hash_table_foreach_remove(vm->theHashTable, clean_it, NULL);
}
    
