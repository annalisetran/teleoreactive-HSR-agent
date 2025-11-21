
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

import java.io.*;
import java.util.Vector;

public class Parser {

    private Scanner scanner;
    private Token curr_token;

    public Parser(String s) {
	StringReader r = new StringReader(s);
	scanner = new Scanner(r);
	curr_token = null;
    }

    private void next_token()
    {
	try {
	    curr_token = scanner.yylex();
	}
	catch (Exception e) {
	    e.printStackTrace();
	}
    }

    private PrologObject parse_list()
    {
	if (curr_token.getType() == Token.CSBRA) {
	    next_token();
	    return new PrologAtom("[]");
	}
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals(","))) {
	    next_token();
	    PrologObject tail = parse_list();
	    if (tail == null) return null;
	    return tail;
	}
	else if (curr_token.getType() == Token.VBAR) {
	    next_token();
	    PrologObject tail = parse_prec700();
	    if ((tail == null) || (curr_token.getType() != Token.CSBRA)) 
		return null;
	    next_token();
	    return tail;
	}
	else {
	    PrologObject head = parse_prec700();
	    if (head == null) return null;
	    PrologObject tail = parse_list();
	    if (tail == null) return null;
	    Vector<PrologObject> cons = new Vector<PrologObject>();
	    cons.add(new PrologAtom("."));
	    cons.add(head);
	    cons.add(tail);
	    return new PrologStruct(cons);
	}
    }
    
    private Boolean parse_args(Vector<PrologObject> args)
    {
	PrologObject t = parse_prec700();
	if (t == null) return false;
	args.add(t);
	while ((curr_token.getType() == Token.ATOM) &&
               (curr_token.getString().equals(","))) {
	    next_token();
	    PrologObject arg = parse_prec700();
	    if (arg == null) return false;
	    args.add(arg);
	}
	return true;
    }

    private PrologObject parse_basic()
    {
	if ((curr_token.getType() == Token.END) ||
	    (curr_token.getType() == Token.CBRA) ||
	    (curr_token.getType() == Token.CSBRA)) {
	    return null;
	}
	else if (curr_token.getType() == Token.OBRA) {
	    next_token();
	    PrologObject t = parse_prec1100();
	    if (curr_token.getType() == Token.CBRA) {
		next_token();
		return t;
	    }
	    else {
		return null;
	    }
	}
	else if (curr_token.getType() == Token.OSBRA) {
	    next_token();
	    return parse_list();
	}
	else if (curr_token.getType() == Token.VAR) {
	    PrologObject t = new PrologVar(curr_token.getString());
	    next_token();
	    return t;
	}
	else if (curr_token.getType() == Token.INT) {
	    PrologObject t = new PrologInt(new Integer(curr_token.getString()));
	    next_token();
	    return t;
	}
	else if (curr_token.getType() == Token.FLOAT) {
	    PrologObject t = 
		new PrologDouble(new Double(curr_token.getString()));
	    next_token();
	    return t;
	}
	else if (curr_token.getType() == Token.STRING) {
	    PrologObject t =  new PrologString(curr_token.getString());
	    next_token();
	    return t;
	}
	else if (curr_token.getType() == Token.ATOM) {
	    PrologObject t = new PrologAtom(curr_token.getString());
	    next_token();
	    if (curr_token.getType() != Token.OBRA) {
		return t;
	    }
	    next_token();
	    Vector<PrologObject> args = new Vector<PrologObject>();
	    args.add(t);
	    if (!parse_args(args) || (curr_token.getType() != Token.CBRA))
		{
		    return null;
		}
	    next_token();
	    return new PrologStruct(args);
	}
	else
	    return null;
    }

    private PrologObject parse_prec50()
    {
	PrologObject t1 = parse_basic();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals(":")))
	    {
		PrologObject op = 
		    new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_basic();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parse_prec100()
    {
	PrologObject t1 = parse_prec50();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    ((curr_token.getString().equals("@"))))
	    {
		PrologObject op = 
		    new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec50();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }



    private PrologObject parse_prec200()
    {
	if (curr_token.getType() == Token.END)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals("-")))
	{
	    PrologObject op = 
		new PrologAtom(curr_token.getString());
	    next_token();
	    PrologObject arg = parse_prec100();
	    if (arg == null) return null;
	    if ((arg.getType() == PrologObject.INT) ||
		(arg.getType() == PrologObject.DOUBLE))
		{
		    arg.negate();
		    return arg;
		}
	    Vector<PrologObject> args = new Vector<PrologObject>();
	    args.add(op);
	    args.add(arg);
		
	    return (new PrologStruct(args));
	}
	PrologObject t1 = parse_prec100();
	if (t1 == null)
	    return null;

	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals("**")))
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_basic();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parse_prec400()
    {
	PrologObject t1 = parse_prec200();
	if (t1 == null)
	    return null;
	while ((curr_token.getType() == Token.ATOM) &&
	    ((curr_token.getString().equals("*")) ||
	     (curr_token.getString().equals("/")) ||
	     (curr_token.getString().equals("//")) ||
	     (curr_token.getString().equals("mod")) ||
	     (curr_token.getString().equals(">>")) ||
	     (curr_token.getString().equals("<<"))
	     )) 
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec200();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		t1 = new PrologStruct(args);
	    }
	return t1;
    }

    private PrologObject parse_prec500()
    {
	PrologObject t1 = parse_prec400();
	if (t1 == null)
	    return null;
	while ((curr_token.getType() == Token.ATOM) &&
	    ((curr_token.getString().equals("+")) ||
	     (curr_token.getString().equals("-")) ||
	     (curr_token.getString().equals("\\/")) ||
	     (curr_token.getString().equals("/\\"))
	     )) 
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec400();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		t1 =  new PrologStruct(args);
	    }
	return t1;
    }

    private PrologObject parse_prec700()
    {
	PrologObject t1 = parse_prec500();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    ((curr_token.getString().equals("=")) ||
	     (curr_token.getString().equals("is")) ||
	     (curr_token.getString().equals(">")) ||
	     (curr_token.getString().equals("<")) ||
	     (curr_token.getString().equals(">=")) ||
	     (curr_token.getString().equals("=<"))
	     )) 
	    {
		PrologObject op =  new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec500();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parse_prec1000()
    {
	PrologObject t1 = parse_prec700();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals(","))) 
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec1000();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parse_prec1050()
    {
	PrologObject t1 = parse_prec1000();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals("->"))) 
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec1050();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parse_prec1100()
    {
	PrologObject t1 = parse_prec1050();
	if (t1 == null)
	    return null;
	if ((curr_token.getType() == Token.ATOM) &&
	    (curr_token.getString().equals(";"))) 
	    {
		PrologObject op = new PrologAtom(curr_token.getString());
		next_token();
		PrologObject t2 = parse_prec1100();
		if (t2 == null)
		    return null;
		Vector<PrologObject> args = new Vector<PrologObject>();
		args.add(op);
		args.add(t1);
		args.add(t2);
		
		return (new PrologStruct(args));
	    }
	return t1;
    }

    private PrologObject parseTerm()
    {
	next_token();
	PrologObject t = parse_prec1100();
	if (curr_token.getType() == Token.END) return t;
	return null;
    }
	

    public PrologObject parse()
    {
	return parseTerm();
    }


}
