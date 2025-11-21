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

import java.util.Vector;

abstract class PrologObject {

    public static int VAR    = 0;
    public static int INT    = 1;
    public static int DOUBLE = 2;
    public static int ATOM   = 3;
    public static int STRING = 4;
    public static int STRUCT = 5;


    abstract public int getType();
    abstract public String toString();
    abstract public void negate();
}

class PrologVar extends PrologObject {

    private String name;

    public PrologVar(String n)
    {
	name = n;
    }

    public int getType()
    {
	return VAR;
    }

    public String toString()
    {
	return name;
    }

    public String getValue()
    {
	return name;
    }

    public void negate()
    {
    }
}

class PrologInt extends PrologObject {

    private Integer value;

    public PrologInt(Integer n)
    {
	value = n;
    }

    public int getType()
    {
	return INT;
    }

    public String toString()
    {
	return value.toString();
    }

    public Integer getValue()
    {
	return value;
    }
    
    public void negate()
    {
	value = -1*value;
    }
}

class PrologDouble extends PrologObject {

    private Double value;

    public PrologDouble(Double n)
    {
	value = n;
    }

    public int getType()
    {
	return DOUBLE;
    }

    public String toString()
    {
	return value.toString();
    }

    public Double getValue()
    {
	return value;
    }

    public void negate()
    {
	value = -1 * value;
    }
}

class PrologAtom extends PrologObject {

    private String name;

    public PrologAtom(String n)
    {
	name = n;
    }

    public int getType()
    {
	return ATOM;
    }

    public String toString()
    {
	return name;
    }

    public String getValue()
    {
	return name;
    }
    public void negate()
    {
    }
}

class PrologString extends PrologObject {

    private String str;

    public PrologString(String n)
    {
	str = n;
    }

    public int getType()
    {
	return STRING;
    }

    public String toString()
    {
	return str;
    }

    public String getValue()
    {
	return str;
    }
    public void negate()
    {
    }
}

class PrologStruct extends PrologObject {

    private Vector<PrologObject> struct;

    public PrologStruct(Vector<PrologObject> s)
    {
	struct = s;
    }

    public int getType()
    {
	return STRUCT;
    }

    public String toString()
    {
	return struct.toString();
    }

    public Vector<PrologObject> getValue()
    {
	return struct;
    }
    public void negate()
    {
    }
}




