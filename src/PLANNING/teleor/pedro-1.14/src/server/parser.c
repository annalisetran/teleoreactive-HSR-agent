
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
#include <glib.h>

#include "parser.h"
#include "token.h"
#include "atomTable.h"
#include "pstack.h"

#include "atoms.h"
extern void logit(char* fmt, ...);
/*
 * The parser for Prolog terms as used in subscriptions and notifications
 * The precedence and associativity of operators follow that of Prolog
 */

static int curr_token_type = ERROR_TOKEN;
static ObjectPtr curr_token = NULL;

void next_token(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  curr_token_type = scanner(heap, atoms, vmap, &curr_token);
}

ObjectPtr parse_basic(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec200(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec400(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec700(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec1000(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec1050(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);
ObjectPtr parse_prec1100(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk);

/*
 * parsing the arguments of a structured term
 * e.g. f(t1, t2)
 */
gboolean parseargs(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  stack_push(stk, NULL);
  if (curr_token_type == CBRA_TOKEN) {
    return TRUE;
  }
  ObjectPtr t = parse_prec700(heap, atoms, vmap, stk);
  if (t == NULL) return FALSE;
  stack_push(stk, t);
  while (curr_token_type == COMMA_TOKEN) {
    next_token(heap, atoms, vmap, stk);
    t = parse_prec700(heap, atoms, vmap, stk);
    if (t == NULL) return FALSE;
    stack_push(stk, t);
  }
  return TRUE;
}

ObjectPtr parse_basic(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  switch (curr_token_type) {
  case ERROR_TOKEN:
  case NEWLINE_TOKEN:
  case COMMA_TOKEN:
  case CBRA_TOKEN:
  case CSBRA_TOKEN:
    return NULL;
    break;

  case OBRA_TOKEN:
    {
      next_token(heap, atoms, vmap, stk);
      ObjectPtr t = parse_prec1100(heap, atoms, vmap, stk);
      if (curr_token_type == CBRA_TOKEN) {
	next_token(heap, atoms, vmap, stk);
	return t;
      }
      else {
	return NULL;
      }
      break;
    }
  
  case OSBRA_TOKEN:
    {
      /* parsing a Prolog list e.g. [],  [e1, e2] */
      next_token(heap, atoms, vmap, stk);
      if (curr_token_type == CSBRA_TOKEN) {
	next_token(heap, atoms, vmap, stk);
	/* inc the refcount for a_nil */
	heapobject* refcount_ptr = (heapobject*)(a_nil) + 1;
	*refcount_ptr += 1;
	return a_nil;
      }
    
      ObjectPtr t = parse_prec700(heap, atoms, vmap, stk);
      if (t == NULL) 
	return NULL;
      ObjectPtr lst = pushList(heap);
      PUT_HEAD(lst, t);
      ObjectPtr lst_tmp = lst;
      while (curr_token_type == COMMA_TOKEN) {
	next_token(heap, atoms, vmap, stk);
	t = parse_prec700(heap, atoms, vmap, stk);
	if (t == NULL)
	  return NULL;
	ObjectPtr tmp = pushList(heap);
	PUT_HEAD(tmp, t);
	PUT_TAIL(lst_tmp, tmp);
	lst_tmp = tmp;
      }
      if (curr_token_type == VBAR_TOKEN) {
	next_token(heap, atoms, vmap, stk);
	t = parse_prec700(heap, atoms, vmap, stk);
	if (t == NULL)
	  return NULL;
	PUT_TAIL(lst_tmp, t);
      }
      else {
	/* inc the refcount for a_nil */
	heapobject* refcount_ptr = (heapobject*)(a_nil) + 1;
	*refcount_ptr += 1;
	PUT_TAIL(lst_tmp, a_nil);
      }
      if (curr_token_type != CSBRA_TOKEN) {
	return NULL;
      }
      next_token(heap, atoms, vmap, stk);
      return lst;
      break;
    }
  case TERM_TOKEN:
    {
      if (!IS_ANY_ATOM(curr_token)) {
	ObjectPtr t = curr_token;
	next_token(heap, atoms, vmap, stk);
	return t;
      }
      ObjectPtr t = curr_token;
      next_token(heap, atoms, vmap, stk);
      if (curr_token_type != OBRA_TOKEN) {
	return t;
      }
      next_token(heap, atoms, vmap, stk);
      if (!parseargs(heap, atoms, vmap, stk) || 
	  (curr_token_type != CBRA_TOKEN)) {
	return NULL;
      }
      next_token(heap, atoms, vmap, stk);
      int arity = 0;
      while (TRUE)
	{
	  if (stack_peek(stk, arity+1) == NULL) break;
	  arity++;
	}
      if (arity == 0) {
        ObjectPtr sterm = pushCompound(heap, 1);
        PUT_ARG(sterm, 1, a_d_none_);
	assert(stack_peek(stk, 1) == NULL);     // the (
	stack_pop(stk);
	PUT_ARG(sterm, 0, t);     // the functor
        return sterm; 
      }
      assert(arity != 0);           // must have at least one arg
      if ((t == a_dot) && (arity == 2)) { // A cons
	ObjectPtr lst = pushList(heap);
	PUT_TAIL(lst, stack_pop(stk));
	PUT_HEAD(lst, stack_pop(stk));
	return lst;
      }
      else {
	ObjectPtr sterm = pushCompound(heap, arity);
	for ( ; arity > 0; arity--)
	  PUT_ARG(sterm, arity, stack_pop(stk));
	assert(stack_peek(stk, 1) == NULL);     // the (
	stack_pop(stk);
	PUT_ARG(sterm, 0, t);     // the functor
	return sterm;
      }
      break;
    }
  default:
    {
      assert(FALSE);
      break;
    }
  }
  return NULL;
}

ObjectPtr parse_prec50(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_basic(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  while ((curr_token == a_colon)) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_basic(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    t1 = sterm;
  }
  return t1;
}
ObjectPtr parse_prec100(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec50(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  while ((curr_token == a_at)) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec50(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    t1 = sterm;
  }
  return t1;
}

ObjectPtr parse_prec200(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  if ((curr_token_type == ERROR_TOKEN) || (curr_token_type == NEWLINE_TOKEN)) {
    return NULL;
  }
  if (curr_token == a_minus) {
    next_token(heap, atoms, vmap, stk);
    ObjectPtr arg = parse_prec100(heap, atoms, vmap, stk);
    if (arg == NULL) 
      return NULL;
    if (IS_INTEGER(arg)) {
      negateInteger(arg);
      return arg;
    }
    if (IS_DOUBLE(arg)) {
      negateDouble(arg);
      return arg;
    }
    ObjectPtr sterm = pushCompound(heap, 1);
    PUT_ARG(sterm, 0, a_minus);
    PUT_ARG(sterm, 1, arg);
    return sterm;
  }
  ObjectPtr t1 = parse_prec100(heap, atoms, vmap, stk);
  if (curr_token == a_power) {
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_basic(heap, atoms, vmap, stk);
    if (t2 == NULL) 
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, a_power);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    return sterm;
  }
  return t1;
}

ObjectPtr parse_prec400(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec200(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  while ((curr_token == a_multiply) ||
	 (curr_token == a_divide) ||
	 (curr_token == a_intdivide) ||
	 (curr_token == a_mod) ||
	 (curr_token == a_shiftl) ||
	 (curr_token == a_shiftr)) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec200(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    t1 = sterm;
  }
  return t1;
}

ObjectPtr parse_prec500(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec400(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  while ((curr_token == a_plus) ||
	 (curr_token == a_minus) ||
	 (curr_token == a_bitwiseand) ||
	 (curr_token == a_bitwiseor)) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec400(heap, atoms, vmap, stk);
    if (t2 == NULL) {
      return NULL;
    }
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    t1 = sterm;
  }
  return t1;
}

ObjectPtr parse_prec700(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec500(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  if ((curr_token == a_eq) || (curr_token == a_is) ||
      (curr_token == a_lt) || (curr_token == a_gt) ||
      (curr_token == a_le) || (curr_token == a_ge)) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec500(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    return sterm;
  }
  return t1;
}

ObjectPtr parse_prec1000(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec700(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  if (curr_token_type == COMMA_TOKEN) {
    ObjectPtr op = a_and;
    inc_atom_refcount(a_and);
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec1000(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    return sterm;
  }
  return t1;
}

ObjectPtr parse_prec1050(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec1000(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  if (curr_token == a_arrow) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec1050(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    return sterm;
  }
  return t1;
}

ObjectPtr parse_prec1100(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  ObjectPtr t1 = parse_prec1050(heap, atoms, vmap, stk);
  if (t1 == NULL)
    return NULL;
  if (curr_token == a_or) {
    ObjectPtr op = curr_token;
    next_token(heap, atoms, vmap, stk);
    ObjectPtr t2 = parse_prec1100(heap, atoms, vmap, stk);
    if (t2 == NULL)
      return NULL;
    ObjectPtr sterm = pushCompound(heap, 2);
    PUT_ARG(sterm, 0, op);
    PUT_ARG(sterm, 1, t1);
    PUT_ARG(sterm, 2, t2);
    return sterm;
  }
  return t1;
}

ObjectPtr 
parseIt(Heap* heap, AtomTable* atoms, VarMap* vmap, Stack* stk)
{
  next_token(heap, atoms, vmap, stk);
  ObjectPtr t = parse_prec1100(heap, atoms, vmap, stk);
  if (curr_token_type != NEWLINE_TOKEN)
    return NULL;
  return t;
}
		
ObjectPtr
parse(char* buff, size_t size, Heap* heap, AtomTable* atoms, 
      VarMap* vmap, Stack* stk)
{
  stack_init(stk);
  varmap_clean(vmap);
  initAtomRef(atoms);
  assert(buff[size] == '\0');
  buff[size+1] = '\0';
  assert(strlen(buff) == size);
  set_buffstate(buff, size+2);

  ObjectPtr term = parseIt(heap, atoms, vmap, stk);


  delete_buffstate();
  varmap_clean(vmap);
  return term;
}

  
