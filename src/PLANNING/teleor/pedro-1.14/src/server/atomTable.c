
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
#include "atomTable.h"
#include "pheap.h"
#include "../lib/pedro_defs.h"

ObjectPtr a_nil;
ObjectPtr a_dot;
ObjectPtr a_subscribe;
ObjectPtr a_unsubscribe;
ObjectPtr a_notify;
ObjectPtr a_register;
ObjectPtr a_deregister;
ObjectPtr a_p2pmsg;
ObjectPtr a_and;
ObjectPtr a_not;
ObjectPtr a_or;
ObjectPtr a_arrow;
ObjectPtr a_if;
ObjectPtr a_prov;
ObjectPtr a_true;
ObjectPtr a_false;
ObjectPtr a_once;
ObjectPtr a_member;
ObjectPtr a_eq;
ObjectPtr a_is;
ObjectPtr a_le;
ObjectPtr a_ge;
ObjectPtr a_lt;
ObjectPtr a_gt;
ObjectPtr a_number;
ObjectPtr a_atom;
ObjectPtr a_string;
ObjectPtr a_list;
ObjectPtr a_split;
ObjectPtr a_splitstring;

ObjectPtr a_colon;
ObjectPtr a_at;
ObjectPtr a_plus;
ObjectPtr a_minus;
ObjectPtr a_multiply;
ObjectPtr a_divide;
ObjectPtr a_intdivide;
ObjectPtr a_mod;
ObjectPtr a_power;
ObjectPtr a_bitwiseand;
ObjectPtr a_bitwiseor;
ObjectPtr a_shiftl;
ObjectPtr a_shiftr;
ObjectPtr a_bitneg;
ObjectPtr a_pi;
ObjectPtr a_e;
ObjectPtr a_abs;
ObjectPtr a_round;
ObjectPtr a_floor;
ObjectPtr a_ceiling;
ObjectPtr a_sqrt;
ObjectPtr a_sin;
ObjectPtr a_cos;
ObjectPtr a_tan;
ObjectPtr a_asin;
ObjectPtr a_acos;
ObjectPtr a_atan;
ObjectPtr a_log;
ObjectPtr a_admin_name;
ObjectPtr a_d_none_;
/*
 * There a 2 kinds of atoms: normal atoms and "dynamic atoms"
 * The dynamic atoms are those that occur in a notifiction but
 * not in the atom table
 * The normal atoms are reference  counted for garbage collection
 */

extern int sizes;

struct _AtomTable
{
  ObjectPtr* atomref;                /* for storing the atoms that are */
                                     /* ref'ed in parsing */
  int reftop;                        /* next index into atomref */
  GHashTable* theHashTable;
};



void delete_key(gpointer str)
{
}

void delete_atom(gpointer atomp)
{
  ObjectPtr obj = (ObjectPtr)atomp;
  g_free(atomp);
}

ObjectPtr add_any_atom(AtomTable* at, Heap* heap, gboolean dynamic, char* key)
{
  assert(at != NULL);
  assert(key != NULL);
  gpointer gobjp = g_hash_table_lookup(at->theHashTable, (gconstpointer)key);
  if (dynamic) {
    if (gobjp == NULL) {
      int size = strlen(key);
      size_t atom_size = SIZEOFATOM + (size >> 2);
      ObjectPtr pos = allocateSpace(heap, atom_size);
      *pos = TYPEDYNATOM | (atom_size << 4);
      *(pos+1) = 0;
      char* cptr = (char*)(pos+2);
      strcpy(cptr, key);
      return pos;
    }
    else {
      return (ObjectPtr)gobjp;
    }
  }
  else {
    if (gobjp == NULL) {
      size_t size = strlen(key);
      size_t atom_size = SIZEOFATOM + (size >> 2);
      gobjp = g_malloc(atom_size*sizeof(heapobject));
      heapobject* hptr = (heapobject*)(gobjp);
      *hptr = TYPEATOM | (atom_size << 4);
      hptr++;
      *hptr = 1;
      char* strptr = (char*)(hptr+1);
      strcpy(strptr, key);
      g_hash_table_insert(at->theHashTable, strptr, gobjp);
    }
    else {
      heapobject* refcount_ptr = (heapobject*)(gobjp) + 1;
      *refcount_ptr += 1;
    }
    pushAtomRef(at, (ObjectPtr)gobjp);
    return (ObjectPtr)gobjp;
  }
}

ObjectPtr add_atom(AtomTable* at, char* key)
{
  assert(at != NULL);
  assert(key != NULL);
  gpointer gobjp = g_hash_table_lookup(at->theHashTable, (gconstpointer)key);

  if (gobjp == NULL) {
    size_t size = strlen(key);
    size_t atom_size = SIZEOFATOM + (size >> 2);
    gobjp = g_malloc(atom_size*sizeof(heapobject));
    heapobject* hptr = (heapobject*)(gobjp);
    *hptr = TYPEATOM | (atom_size << 4);
    hptr++;
    *hptr = 1;
    char* strptr = (char*)(hptr+1);
    strcpy(strptr, key);
    g_hash_table_insert(at->theHashTable, strptr, gobjp);
  }
  else {
    heapobject* refcount_ptr = (heapobject*)(gobjp) + 1;
    *refcount_ptr += 1;
  }
  return (ObjectPtr)gobjp;
}

void inc_atom_refcount(ObjectPtr gobjp)
{
  assert(IS_ATOM(gobjp));
  heapobject* refcount_ptr = (heapobject*)(gobjp) + 1;
  *refcount_ptr += 1;

}

void atom_table_destroy(AtomTable* at)
{
  g_hash_table_destroy(at->theHashTable);
  g_free(at->atomref);
  g_free(at);
}

void initAtomRef(AtomTable* at)
{
  at->reftop = 0;
}

void pushAtomRef(AtomTable* at, ObjectPtr refptr)
{
  at->atomref[at->reftop] = refptr;
  at->reftop++;
}

void dec_all_atom_ref(AtomTable* at)
{
  int i;
  int top = at->reftop;
  ObjectPtr* stack = at->atomref;
  for (i = 0; i < top; i++) {
    dec_atom_refcount(at, stack[i]);
  }
}
      
/*
 * The commonly occurring atoms are pre defined
 */
AtomTable* atom_table_new()
{
  AtomTable* at = g_new (AtomTable, 1);
  at->atomref = g_malloc(sizes);
  at->reftop = 0;
  at->theHashTable = g_hash_table_new_full(g_str_hash, g_str_equal, 
					   NULL, delete_atom);
  a_nil = add_atom(at, "[]");
  a_dot = add_atom(at, ".");
  a_subscribe = add_atom(at, "subscribe");
  a_unsubscribe = add_atom(at, "unsubscribe");
  a_notify = add_atom(at, "notify");
  a_register = add_atom(at, "register");
  a_deregister = add_atom(at, "deregister");
  a_p2pmsg = add_atom(at, "p2pmsg");
  a_and = add_atom(at, "','");
  a_not = add_atom(at, "not");
  a_or = add_atom(at, ";");
  a_arrow = add_atom(at, "->");
  a_if = add_atom(at, "if");
  a_prov = add_atom(at, ":-");
  a_true = add_atom(at, "true");
  a_false = add_atom(at, "fail");
  a_once = add_atom(at, "once");
  a_member = add_atom(at, "member");
  a_eq = add_atom(at, "=");
  a_is = add_atom(at, "is");
  a_le = add_atom(at, "=<");
  a_ge = add_atom(at, ">=");
  a_lt = add_atom(at, "<");
  a_gt = add_atom(at, ">");
  a_number = add_atom(at, "number");
  a_atom = add_atom(at, "atom");
  a_string = add_atom(at, "string");
  a_list = add_atom(at, "list");
  a_split = add_atom(at, "split");
  a_splitstring = add_atom(at, "splitstring");

  a_colon = add_atom(at, ":");
  a_at = add_atom(at, "@");
  a_plus = add_atom(at, "+");
  a_minus = add_atom(at, "-");
  a_multiply = add_atom(at, "*");
  a_divide = add_atom(at, "/");
  a_intdivide = add_atom(at, "//");
  a_mod = add_atom(at, "mod");
  a_power = add_atom(at, "**");
  a_bitwiseand = add_atom(at, "/\\");
  a_bitwiseor = add_atom(at, "\\/");
  a_shiftl = add_atom(at, "<<");
  a_shiftr = add_atom(at, ">>");
  a_bitneg = add_atom(at, "\\");
  a_pi = add_atom(at, "pi");
  a_e = add_atom(at, "e");
  a_abs = add_atom(at, "abs");
  a_round = add_atom(at, "round");
  a_floor = add_atom(at, "floor");
  a_ceiling = add_atom(at, "ceiling");
  a_sqrt = add_atom(at, "sqrt");
  a_sin = add_atom(at, "sin");
  a_cos = add_atom(at, "cos");
  a_tan = add_atom(at, "tan");
  a_asin = add_atom(at, "asin");
  a_acos = add_atom(at, "acos");
  a_atan = add_atom(at, "atan");
  a_log = add_atom(at, "log");
  a_admin_name = add_atom(at, "admin");
  a_d_none_ = add_atom(at, "'$none_'");
  return at;
}

void dec_atom_refcount(AtomTable* at, ObjectPtr t)
{
  heapobject* refptr = (heapobject*)(t + 1);
  assert(*refptr > 0);
  *refptr -= 1;
  if (*refptr == 0) {
    gboolean removed = 
      g_hash_table_remove(at->theHashTable, (char*)(refptr+1));
    assert(removed);
  }
}

/*
 * decrement all occurrences of all normal atoms occurring in the term 
 * objptr
 */
void dec_refcount_all_atoms(AtomTable* at, ObjectPtr objptr)
{
  ObjectPtr t = objptr;
  switch (GET_TYPE(t)) {
  case TYPEVAR:
  case TYPEINTEGER:
  case TYPEDOUBLE:
  case TYPESTRING:
    return;
    break;
  case TYPEATOM:
    {
      dec_atom_refcount(at, t);
      return;
      break;
    }
  case TYPELIST:
    {
      dec_refcount_all_atoms(at, GET_HEAD(t));
      dec_refcount_all_atoms(at, GET_TAIL(t));
      return;
      break;
    }
  case TYPECOMPOUND:
    {
      int arity = GET_TAG_INFO(t);
      int i;
      for (i = 0; i <= arity; i++)
        dec_refcount_all_atoms(at, GET_ARG(t, i));
    }
    return;
    break;
  default:
    assert(FALSE);
  }
}
