
%%
%class Scanner
%unicode
%type Token


WHITE_SPACE_CHAR=[\r\ \t\b\012]
%%

[+-]?[0-9]+"."[0-9]+([eE][+-]?[0-9]+)?  { return new Token(yytext(), Token.FLOAT); }
[+-]?[0-9]+([eE][+-]?[0-9]+)	  { return new Token(yytext(), Token.FLOAT); }


[+-]?[0-9]+	{ return new Token(yytext(), Token.INT); }

"("             { return new Token(yytext(), Token.OBRA); }
")"             { return new Token(yytext(), Token.CBRA); }
"["             { return new Token(yytext(), Token.OSBRA); }
"]"             { return new Token(yytext(), Token.CSBRA); }
","             { return new Token(yytext(), Token.ATOM); }
"|"             { return new Token(yytext(), Token.VBAR); }

[A-Z][A-Za-z0-9_]* { return new Token(yytext(), Token.VAR); }

\"[^\"\\]*(\\.[^\"\\]*)*\" { return new Token(yytext(), Token.STRING); }

[a-z][A-Za-z0-9_]*         { return new Token(yytext(), Token.ATOM); }
'[^'\\]*(\\.[^'\\]*)*'   { return new Token(yytext(), Token.ATOM); }
[-/+*<=>#@$\\\^&~`:.?!;]+   { return new Token(yytext(), Token.ATOM); }
{} { return new Token(yytext(), Token.ATOM); }

"\n" { return new Token(yytext(), Token.END); }

{WHITE_SPACE_CHAR}+ { }

