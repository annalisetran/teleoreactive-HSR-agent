// string_escapes.cc - Get string from a string stream.
//
// ##Copyright##
// 
// Copyright 2000-2016 Peter Robinson  (pjr@itee.uq.edu.au)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.00 
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ##Copyright##
//
// $Id: string_escapes.cc,v 1.7 2005/03/08 00:35:14 qp Exp $

#include <sstream>

#include "atom_table.h"
#include "io_qp.h"
#include "thread_qp.h"
#include "hash_qp.h"
#ifdef WIN32
  #define PCRE2_STATIC 1
  #define PCRE2_CODE_UNIT_WIDTH 8
  #include <string>
  #include <iostream>
  #include <regex.h>
  #include "pcre2.h"
#else
#if defined(PCRE)
  #define PCRE2_CODE_UNIT_WIDTH 8
  #include <regex.h>
  #include <pcre2.h>
#endif
#endif

extern AtomTable *atoms;
extern IOManager *iom;

//
// psi_stream_to_chars(stream_index, variable)
// Get the list of characters from a string stream.
//
Thread::ReturnValue
Thread::psi_stream_to_chars(Object *& stream_arg,
			    Object *& char_list_arg)
{
  Object* stream_object = heap.dereference(stream_arg);
  
  QPStream *stream;
  DECODE_STREAM_OUTPUT_ARG(heap, *iom, stream_object, 1, stream);

  //
  // Converting the string to a list of characters.
  //
  Object* tail = AtomTable::nil;
  int size = static_cast<int>(stream->str().length());
  if (size > 0)
    {
      string data = stream->str();
      for (int i = size; i > 0; i--)
	{
	  Cons* temp = heap.newCons();
	  temp->setTail(tail);
	  temp->setHead(heap.newInteger(data[i - 1]));
	  tail = temp;
	}
    }
  
  //
  // Return the list of characters.
  //
  char_list_arg = tail;

  return RV_SUCCESS;
}

//
// psi_stream_to_atom(stream_index, variable)
// Get an atom from a string stream.
//
Thread::ReturnValue
Thread::psi_stream_to_atom(Object *& stream_arg, Object *& atom_arg)
{
  Object* stream_object = heap.dereference(stream_arg);
  
  QPStream *stream;
  DECODE_STREAM_OUTPUT_ARG(heap, *iom, stream_object, 1, stream);

  //
  // Return the atom.
  //
  int size = static_cast<int>(stream->str().length());
  if (size == 0)
    {
      atom_arg = atoms->add("");
    }
  else
    {
      string data = stream->str();
      char* buff = new char[size+1];
      for (int i = 0; i < size; i++)
	{
	  buff[i] = data[i];
	}
      buff[size] = '\0';
      atom_arg = atoms->add(buff);
      delete buff;
    }

  return RV_SUCCESS;
}


//
// psi_stream_to_string(stream_index, variable)
// Get a string from a string stream.
//
Thread::ReturnValue
Thread::psi_stream_to_string(Object *& stream_arg, Object *& string_arg)
{
  Object* stream_object = heap.dereference(stream_arg);
  QPStream *stream;
  DECODE_STREAM_OUTPUT_ARG(heap, *iom, stream_object, 1, stream);

  //
  // Return the atom.
  //
  int size = static_cast<int>(stream->str().length());
  string data = stream->str();
  char* buff = new char[size+1];
  for (int i = 0; i < size; i++)
    {
      buff[i] = data[i];
    }
  buff[size] = '\0';
  string_arg = heap.newStringObject(buff);
  delete buff;

  return RV_SUCCESS;
}


//
// psi_list_to_string(list, variable)
// Convert a list into a string
//
Thread::ReturnValue
Thread::psi_list_to_string(Object *& list_arg, Object *& string_arg)
{
  Object* list_object = heap.dereference(list_arg);

  if (list_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  string lchars;
  while (!list_object->isNil())
    {
      if (!list_object->isCons())
	{
	  PSI_ERROR_RETURN(EV_TYPE, 1);
	}
      Cons* lst = OBJECT_CAST(Cons*, list_object);
      Object* head = lst->getHead()->variableDereference();
      if (!head->isInteger())
	{
	  PSI_ERROR_RETURN(EV_TYPE, 1);
	}
      int v = head->getInteger();
      if ((v < 0) || (v > 127))
	{
	  PSI_ERROR_RETURN(EV_TYPE, 1);
	}
      lchars.push_back((char)v);
      list_object = lst->getTail()->variableDereference();
    }
  string_arg = heap.newStringObject(lchars.c_str());
  return RV_SUCCESS;
}

//
// psi_string_to_list(string, variable)
// Convert a string into a list
//
Thread::ReturnValue
Thread::psi_string_to_list(Object *& string_arg, Object *& list_arg)
{
  Object* string_object = heap.dereference(string_arg);

  if (string_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  StringObject* so = OBJECT_CAST(StringObject*, string_object);
  list_arg = heap.stringToList(so->getChars());
  return RV_SUCCESS;
}


//
// psi_string_to_atom(string, variable)
// Convert a string into an atom
//
Thread::ReturnValue
Thread::psi_string_to_atom(Object *& string_arg, Object *& atom_arg)
{
  Object* string_object = heap.dereference(string_arg);

  if (string_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  if (!string_object->isString())   
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  atom_arg = atoms->add(OBJECT_CAST(StringObject*, string_object)->getChars());
  return RV_SUCCESS;
}

//
// psi_atom_to_string(atom, variable)
// Convert an atom into a string
//
Thread::ReturnValue
Thread::psi_atom_to_string(Object *& atom_arg, Object *& string_arg)
{
  Object* atom_object = heap.dereference(atom_arg);

  if (atom_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  if (!atom_object->isAtom())   
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  char* atomstring = OBJECT_CAST(Atom*, atom_object)->getName();

  string_arg = heap.newStringObject(atomstring);

  return RV_SUCCESS;
}

//
// psi_string_length(string, variable)
// Get the length of a string
//
Thread::ReturnValue
Thread::psi_string_length(Object *& string_arg, Object *& length_arg)
{
  Object* string_object = heap.dereference(string_arg);

  if (string_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  if (!string_object->isString())   
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  length_arg = heap.newInteger(strlen(OBJECT_CAST(StringObject*, string_object)->getChars()));

  return RV_SUCCESS;
}

//
// psi_string_concat(string, string, variable)
// Concatentate 2 strings
//
Thread::ReturnValue
Thread::psi_string_concat(Object *& string1_arg, Object *& string2_arg, 
			  Object *& concat_arg)
{
  Object* string1_object = heap.dereference(string1_arg);
  Object* string2_object = heap.dereference(string2_arg);

  if (string1_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  if (string2_object->isVariable())
    {
      PSI_ERROR_RETURN(EV_INST, 2);
    }

  if (!string1_object->isString())   
    {
      PSI_ERROR_RETURN(EV_INST, 1);
    }
  if (!string2_object->isString())   
    {
      PSI_ERROR_RETURN(EV_INST, 2);
    }

  string result(OBJECT_CAST(StringObject*, string1_object)->getChars());

  result.append(OBJECT_CAST(StringObject*, string2_object)->getChars());

  concat_arg = heap.newStringObject(result.c_str());
  return RV_SUCCESS;
}

//
// psi_split_string(string, integer, variable, variable)
// Split a string at the given position
//
Thread::ReturnValue
Thread::psi_split_string(Object *& string_arg, Object *& pos_arg, 
			 Object *& split1_arg, Object *& split2_arg)
{
  Object* string_object = heap.dereference(string_arg);
  int pos = pos_arg->variableDereference()->getInteger();

  string stringstring(OBJECT_CAST(StringObject*, string_object)->getChars());
  string substr1 = stringstring.substr(0, pos);
  string substr2 = stringstring.substr(pos);

  split1_arg = heap.newStringObject(substr1.c_str());
  split2_arg = heap.newStringObject(substr2.c_str());
  return RV_SUCCESS;
}


//
// psi_hash_string(A,B)
// mode(in,out)
// 
//
Thread::ReturnValue
Thread::psi_hash_string(Object *& object1, Object *& object2)
{
  PrologValue pval1(object1);
  heap.prologValueDereference(pval1);
  Object* dval = pval1.getTerm();

  if (!dval->isString())
    {
      PSI_ERROR_RETURN(EV_TYPE, 1);
    }

  word32 v = Hash(OBJECT_CAST(StringObject*, dval)->getChars());
  object2 = heap.newInteger(v);
  return RV_SUCCESS;
}



Thread::ReturnValue
Thread::psi_re_match(Object *& object1, Object *& object2,
                     Object *& object3, Object *& object4, Object *& object5)
{
#if defined(PCRE) ||  defined(WIN32)

  // The first arg is the RE string
  Object* string1_object = heap.dereference(object1);
  PCRE2_SPTR restring = (PCRE2_SPTR)(OBJECT_CAST(StringObject*, string1_object)->getChars());

  int error;
  PCRE2_SIZE erroffset;
  // Compile the RE string (rptr needs to be freed when finished)
  pcre2_code* rptr = pcre2_compile(restring,
                                   PCRE2_ZERO_TERMINATED,
                                   0,                 
                                   &error,           
                                   &erroffset,      
                                   NULL);              

  // cannot compile RE string
  if (rptr == NULL) {
    return(RV_FAIL);
  }

  // The second argument is the string to match RE against
  // On backtracking this string is the original string with the
  // initial matched string removed
  Object* string2_object = heap.dereference(object2);
  PCRE2_SPTR subject = (PCRE2_SPTR)(OBJECT_CAST(StringObject*, string2_object)->getChars());
  // The third arg is the starting position of the  search
  // Initially 0 and is the position relative to the original string
  int startpos = object5->variableDereference()->getInteger();
  PCRE2_SIZE *ovector;
  pcre2_match_data *match_data;

  PCRE2_SIZE subject_length = (PCRE2_SIZE)strlen((char *)subject);

  // Create match data to be used in pcre2_match
  // needs to be freed when finished
  match_data = pcre2_match_data_create_from_pattern(rptr, NULL);

  // do the search
  int rc = pcre2_match(rptr,                   
                       subject,             
                       subject_length,      
                       0,                    
                       0,
                       match_data,
                       NULL);
  
  if (rc < 0) {
    // no matches or error - free data and fail
    pcre2_match_data_free(match_data); 
    pcre2_code_free(rptr);
    return RV_FAIL;
  }
  if (rc == 0) {
    // vector of offsets too small - free data and fail
    pcre2_match_data_free(match_data); 
    pcre2_code_free(rptr);
    return RV_FAIL;
  }

  ovector = pcre2_get_ovector_pointer(match_data);
  // no longer need match_data - free it
  pcre2_match_data_free(match_data);

  // build up the match list - a list of start-end indices with the
  // first being the overall match range
  Cons* list = heap.newCons();
  object4 = list;
  for (int i = 0; i < rc; i++) {
    int start;
    int finish;
    if (ovector[2*i] == ovector[2*i+1]) {
      // empty match - set both to startpos
      start = startpos + ovector[0];
      finish = start;
    } else {
      start = ovector[2*i]+startpos;
      finish = ovector[2*i+1]+startpos;
    }
    Structure* sterm = heap.newStructure(2);
    sterm->setFunctor(AtomTable::colon);
    sterm->setArgument(1, heap.newInteger(start));
    sterm->setArgument(2, heap.newInteger(finish));
    list->setHead(sterm);
    if (i == (rc - 1)) {
      list->setTail(AtomTable::nil);
      break;
    }
    Cons* list_tmp = heap.newCons();
    list->setTail(list_tmp);
    list = list_tmp;
  }
  if (ovector[0] != ovector[1] || subject_length > 0) {
    subject += ovector[1];
  }
  object3 = heap.newStringObject((char *)subject);
  pcre2_code_free(rptr);
  return RV_SUCCESS;
#else
  cerr << "PCRE library not installed" << endl;
  return RV_FAIL;
#endif
}
