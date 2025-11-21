/*
  Copyright (C) 2011 Peter Robinson
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
 * Support for the Prolog tokenizer used in client_api.c
 */

void set_buffstate(char* buff, int size);

void delete_buffstate();

extern char *yytext;
extern int yylex();

#define ERROR_TOKEN              0
#define NEWLINE_TOKEN            1
#define OBRA_TOKEN               2
#define CBRA_TOKEN               3
#define COMMA_TOKEN              4
#define OSBRA_TOKEN              5
#define CSBRA_TOKEN              6
#define VBAR_TOKEN               7
#define INT_TOKEN                8
#define DOUBLE_TOKEN             9
#define ATOM_TOKEN               10
#define STRING_TOKEN             11
#define VAR_TOKEN                12
