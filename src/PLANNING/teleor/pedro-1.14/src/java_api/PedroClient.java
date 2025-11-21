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
import java.net.*;
import java.util.concurrent.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class PedroClient {
    private Socket ackSocket = null;
    private Socket pedroSocket = null;
    private BufferedReader ack_reader = null;
    private BufferedWriter ack_writer = null;
    private BufferedReader reader = null;
    private BufferedWriter writer = null;
    private String line;
    private int port;
    private String machine;
    private String my_machine_name;
    private String name;
    private String host;
    private BlockingQueue<String> notify_queue;
    private BlockingQueue<Integer> subscription_queue;
    private Pipe pipe;
    private List _listeners = new ArrayList();

    public PedroClient() {
	port = 4550;
	machine = "127.0.0.1";
	connect();
    }

    public PedroClient(int p) {
	port = p;
	machine = "127.0.0.1";
	connect();
    }

    public PedroClient(String h) {
	port = 4550;
	machine = h;
	connect();
    }

    public PedroClient(int p, String h) {
	port = p;
	machine = h;
	connect();
    }



    public void connect() {
	if (pedroSocket != null) {
	    System.err.println("Already connected");
	}
	else {
	    try {
		Socket infosock = new Socket(machine, port);
		InputStreamReader info_isr 
		    = new InputStreamReader(infosock.getInputStream());
		BufferedReader inforeader  = new BufferedReader(info_isr);
		String infoline = inforeader.readLine();
		String [] parts = infoline.split(" ");
		machine = parts[0];
		int ack_port = Integer.parseInt(parts[1]);
		int data_port = Integer.parseInt(parts[2]);
		infosock.close();
		    
		 // set up ack socket
		ackSocket = new Socket(machine, ack_port);
		OutputStreamWriter ack_osw 
		    = new OutputStreamWriter(ackSocket.getOutputStream());
		InputStreamReader ack_isr 
		    = new InputStreamReader(ackSocket.getInputStream());
		ack_writer = new BufferedWriter(ack_osw);
		ack_reader = new BufferedReader(ack_isr);
                // Read client ID
                Long id = new Long(ack_reader.readLine());

                // set up data socket
		pedroSocket = new Socket(machine, data_port);
		OutputStreamWriter osw 
		    = new OutputStreamWriter(pedroSocket.getOutputStream());
		InputStreamReader isr 
		    = new InputStreamReader(pedroSocket.getInputStream());
		writer = new BufferedWriter(osw);
		reader = new BufferedReader(isr);
                
                writer.write(id + "\n");
                writer.flush();
                String line = reader.readLine();
                if (!line.equals( "ok")) {
                    System.err.println("Can't complete connection");
                } 
		notify_queue = new LinkedBlockingQueue<String>();
		pipe = new Pipe(notify_queue, reader, _listeners);
                InetAddress inet = ackSocket.getLocalAddress();
		my_machine_name = inet.getHostAddress();
		//System.err.println("host = " + host);
                // check if host has the same IP address as the socket
                //InetAddress inet2 = InetAddress.getByName(host);
                //if (inet.getHostAddress() != inet2.getHostAddress())
                //    host = inet.getHostAddress();
	    }
	    catch (UnknownHostException e) {
		System.err.println("Can't find localhost");
	    }
	    catch (IOException e) {
		System.err.println("Can't open streams");
	    }	
	}
    }

    
    public synchronized void addPedroListener( PedroListener l ) {
        _listeners.add( l );
    }
    
    public synchronized void removePedroListener( PedroListener l ) {
        _listeners.remove( l );
    }
     

    public void disconnect() {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	}
	else {
	    try {
		pipe.stop = true;
		pedroSocket.close();
		pedroSocket = null;
	    }
	    catch (UnknownHostException e) {
		System.err.println("Problem closing socket");
	    }
	    catch (IOException e) {
		System.err.println("Problem closing streams");
	    }	
	}
    }

    public int subscribe(String term, String goal) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else {
	    try {
		writer.write("subscribe(" + term + ", (" + goal + "), 0)\n");
		writer.flush();
	    }
	    catch (IOException e) {
		System.err.println("subscribe: Can't write");
	    }
	}
	Integer id = null;
	try {
	    id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("subscribe: Can't read");
	    System.exit(-1);
	}
	return(id);

    }



    public int unsubscribe(int id) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else {
	    try {
		writer.write("unsubscribe(" + id + ")\n");
		writer.flush();
	    }
	    catch (IOException e) {
		System.err.println("unsubscribe: Can't write");
	    }   
	}
	Integer ack_id = null;
	try {
	    ack_id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("unsubscribe: Can't read");
	    System.exit(-1);
	}
	return(ack_id);
    }

    public int notify(String term) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else {
	    try {
		writer.write(term + "\n");
		writer.flush();
	    }
	    catch (IOException e) {
		System.err.println("notify: Can't write");
	    }   
	}
	Integer id = null;
	try {
	    id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("notify: Can't read");
	    System.exit(-1);
	}
	return(id);
    }

    public String get_notification() {
	String notification = "";
	try {
	    notification = notify_queue.take();
	}
	catch (InterruptedException e) {
	    System.err.println("get_notification failed");
	}
	return notification;   
    }
    
    public boolean notification_ready() {
        return notify_queue.peek() != null;
    }
    
    public int register(String n) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else if (name != null) {
	    System.err.println("Already registered");
	    return 0; 
	}
	else {
	    try {
		writer.write("register(" + n + ")\n");
		writer.flush();
	    }
	    catch (IOException e) {
		System.err.println("register: Can't write");
	    }   
	}
	Integer id = null;
	try {
	    id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("notify: Can't read");
	    System.exit(-1);
	}
	if (id != 0) {
	    name = n;
	}
	return(id);
    }
    
    public int deregister(String n) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else if (name == null) {
	    System.err.println("Not registered");
	    return 0; 
	}
	else {
	    try {
		writer.write("deregister(" + name + ")\n");
		writer.flush();
	    }
	    catch (IOException e) {
		System.err.println("register: Can't write");
	    }   
	}
	Integer id = null;
	try {
	    id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("notify: Can't read");
	    System.exit(-1);
	}
	name = null;
	return(id);
    }

    public int p2p(String toaddr, String msg) {
	if (pedroSocket == null) {
	    System.err.println("Not connected");
	    return 0;
	}
	else if (name == null) {
	    System.err.println("Not registered");
	    return 0; 
	}
	else {
	    try {
               toaddr = toaddr.replace("localhost", "'"+my_machine_name+"'");
                if (toaddr.indexOf('@') == -1) {
                    if (toaddr.matches("^[_A-Z][^:]*$")) {
                            writer.write("p2pmsg(" + toaddr +
                                         ", " + name + "@'"
                                         + my_machine_name + "', " + msg + ")\n");
                            writer.flush();
                        } else {
                            writer.write("p2pmsg(" + toaddr + "@'"
                                         + my_machine_name + "', " + name + "@'"
                                         + my_machine_name + "', " + msg + ")\n");
                            writer.flush();
                        }
                }
                else {
                  writer.write("p2pmsg(" + toaddr + ", " + name + "@'"
                                 + my_machine_name + "', " + msg + ")\n");
                    writer.flush();
                }
	    }
	    catch (IOException e) {
		System.err.println("register: Can't write");
	    }   
	}
	Integer id = null;
	try {
	    id = new Integer(ack_reader.readLine());
	}
	catch (IOException e) {
	    System.err.println("notify: Can't read");
	    System.exit(-1);
	}
	return(id);
    }



    class Pipe implements Runnable {
	public boolean stop;
	private Thread thr;
	private BlockingQueue<String> notify_queue;
	BufferedReader reader;
	public Pipe(BlockingQueue<String> q, BufferedReader r, 
		    List listeners) {
	    stop = false;
	    notify_queue = q;
	    reader = r;
	    thr = new Thread(this);
	    thr.start();
	    _listeners = listeners;
	}
	
	private String get_notification() {
	    try {
		line = reader.readLine();
		return line + "\n";
	    }
	    catch (IOException e) {
		return null;
	    }
	}

	private synchronized void _firePedroEvent() {
	    PedroEvent ev = new PedroEvent( this);
	    Iterator listeners = _listeners.iterator();
	    while(listeners.hasNext() ) {
		( (PedroListener) listeners.next() ).pedroReceived( ev );
	    }
	}

	public void run() {
	    try {
		while (!stop) {
		    String notf = get_notification();
		    if (notf != null) {
			notify_queue.put(notf);
			_firePedroEvent();
		    }
		}
	    } catch (InterruptedException ex) {
		
	    }
	}
    }


}



