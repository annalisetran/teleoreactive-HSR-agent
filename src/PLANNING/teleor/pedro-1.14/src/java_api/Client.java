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


public class Client {

  public static void main(String [] args)
    {
	PedroClient client = new PedroClient();
	System.out.println("ID = " + client.subscribe(args[0], args[1]));
	client.register("javaclient");
	while (true) {
	    String msg = client.get_notification();
	    int index = msg.indexOf(' ');
	    Parser parser = new Parser(msg.substring(index+1));
	    System.out.println(parser.parse());
	}
    }
}
