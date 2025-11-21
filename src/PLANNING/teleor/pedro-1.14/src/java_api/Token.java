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


public class Token {
    public static int END    = 0;
    public static int FLOAT  = 1;
    public static int INT    = 2;
    public static int OBRA   = 3;
    public static int CBRA   = 4;
    public static int OSBRA  = 5;
    public static int CSBRA  = 6;
    public static int COMMA  = 7;
    public static int VBAR   = 8;
    public static int VAR    = 9;
    public static int STRING = 10;
    public static int ATOM   = 11;


    private String str;
    private int type;

    public Token(String s, int t)
    {
	str = s;
	type = t;
    }

    public int getType()
    {
	return type;
    }

    public String getString()
    {
	return str;
    }

}
