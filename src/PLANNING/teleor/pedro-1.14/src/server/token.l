
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

%{
#include <math.h>
#include "objects.h"
#include "pheap.h"
#include "atomTable.h"
#include "varMap.h"
#include "token.h"

YY_BUFFER_STATE buffstate;

int
atomNeedsQuotes(const char *s);

int makeAtomsStatic = 0;
int firstToken = 1;

#define YY_DECL int scanner(Heap* heap, AtomTable* atoms, VarMap* vmap, ObjectPtr* val)
%}

%option noyywrap

DIGIT [0-9]
LOWER [a-z]
UPPER [A-Z]

%%
[0-9]+"."[0-9]+("e"|"e+"|"e-")[0-9]+  { *val = pushDouble(heap, atof(yytext)); firstToken = 0; return TERM_TOKEN; }
[0-9]+"."[0-9]+("E"|"E+"|"E-")[0-9]+  { *val = pushDouble(heap, atof(yytext)); firstToken = 0; return TERM_TOKEN; }
[0-9]+("e"|"e+"|"e-")[0-9]+  { *val = pushDouble(heap, atof(yytext)); firstToken = 0; return TERM_TOKEN; }
[0-9]+("E"|"E+"|"E-")[0-9]+  { *val = pushDouble(heap, atof(yytext)); firstToken = 0; return TERM_TOKEN; }
[0-9]+"."[0-9]+  { *val = pushDouble(heap, atof(yytext)); firstToken = 0; return TERM_TOKEN; }

[0-9]+    { *val = pushInteger(heap, atoll(yytext)); firstToken = 0; return TERM_TOKEN; }





"("        { firstToken = 0; *val = NULL; return OBRA_TOKEN; }
")"        { firstToken = 0; *val = NULL; return CBRA_TOKEN; }
","        { firstToken = 0; *val = NULL; return COMMA_TOKEN; }
"["        { firstToken = 0; *val = NULL; return OSBRA_TOKEN; }
"]"        { firstToken = 0; *val = NULL; return CSBRA_TOKEN; }
"|"        { firstToken = 0; *val = NULL; return VBAR_TOKEN; }


"subscribe"   {  *val = add_atom(atoms, yytext); 
                 if (firstToken) makeAtomsStatic = 1; 
                 firstToken = 0;
                 return TERM_TOKEN; }
"register"   {  *val = add_atom(atoms, yytext); 
                 if (firstToken) makeAtomsStatic = 1; 
                 firstToken = 0;
                 return TERM_TOKEN; }

[_A-Z][A-Za-z0-9_]*  {  *val = varmap_add(vmap, yytext); 
                      firstToken = 0; return TERM_TOKEN; }

[a-z][A-Za-z0-9_]*  { *val = add_any_atom(atoms, heap, (makeAtomsStatic == 0), yytext); firstToken = 0; return TERM_TOKEN; }

"'"[^'\\]*("\\".[^'\\]*)*"'"  { if(atomNeedsQuotes(yytext))
                        {
                           *val = add_any_atom(atoms, heap, (makeAtomsStatic == 0), yytext);
                        }
                       else
                        {
                          int size = strlen(yytext);
                          char* copy = (char*)g_malloc(size);
                          strncpy(copy, yytext+1, size-2);
                          copy[size-2] = '\0';
                          *val = add_any_atom(atoms, heap, (makeAtomsStatic == 0), copy); 
                          g_free(copy);
                         }
                       firstToken = 0; return TERM_TOKEN; 
                     }

"\""[^\"\\]*("\\".[^\"\\]*)*"\""  { *val = pushNewString(heap, yytext); firstToken = 0; return TERM_TOKEN; }

";" { *val = add_any_atom(atoms, heap, (makeAtomsStatic == 0), yytext); firstToken = 0; return TERM_TOKEN; }

[-/+*<=>#@$\\^&~`:.?!]+ { *val = add_any_atom(atoms, heap, (makeAtomsStatic == 0), yytext); firstToken = 0; return TERM_TOKEN; }

"{}" { *val = add_any_atom(atoms, heap, FALSE, yytext); firstToken = 0; return TERM_TOKEN; }

"\n" { return NEWLINE_TOKEN; }

[ \t]+

. { return ERROR_TOKEN; } 

%%

void set_buffstate(char* buff, int size)
{
    makeAtomsStatic = 0;
    firstToken = 1;
    buffstate = yy_scan_buffer(buff, size);
}

void delete_buffstate()
{
    yy_delete_buffer(buffstate);
}

int
atomNeedsQuotes(const char *s)
{
  s++;
  if (strcmp(s, "'") == 0)
    {
      return(1);
    }
  else if ((strcmp(s, "[]'") == 0) ||
           (strcmp(s, ";'") == 0))
    {
      return(0);
    }
  else if (strchr("abcdefghijklmnopqrstuvwxyz", *s) != NULL)
    {
      s++;
      while(*s != '\0')
        {
          if (strchr("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789", *s) !=
          NULL)
            {
              //
              // index finds the given character in the 'safe'
              // range.  So check the next character.
              // 
              s++;
            }
          else
            {
              return (strcmp(s, "'") != 0);
            }
        }
      return(0);
    }
  else
    {
      while(*s != '\0')
        {
          if (strchr("#$&*+-./:<=>?@^~\\", *s) != NULL)
            {
              //
              // index finds the given character in the 'safe'
              // range.  So check the next character.
              // 
              s++;
            }
          else
            {
              return (strcmp(s, "'") != 0);
            }
        }
      return(0);
    }
}

