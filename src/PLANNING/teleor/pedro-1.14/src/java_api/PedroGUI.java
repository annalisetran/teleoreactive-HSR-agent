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

import java.awt.*;
import java.awt.event.*;
import javax.swing.event.*;

import javax.swing.*;
import java.util.Vector;
import javax.swing.table.*;

public class PedroGUI extends JFrame implements ActionListener 
 {
     
     JTextArea messages;
     JTable subscriptions;
     JTextField subtermtext;
     JTextField subgoaltext;
     JButton subsubmit;
     JButton subunsubmit;
     JButton notifysend;
     JTextField notifytext;
     JTextField registertext;
     JButton register_button;
     JButton unregister_button;
     JTextField p2pmsgtext;
     JTextField p2paddrtext;
     JButton p2psend_button;
     boolean registered;
     String name;
     int selected;
     
     DefaultTableModel subsmodel = new DefaultTableModel();
     
     
     PedroClient client;
     public PedroGUI(String machine) {
	 if (machine.equals("")) {
	     client = new PedroClient();
	 } else {
	     client = new PedroClient(machine);
	 }
         registered = false;
         selected = -1;
         subsmodel.addColumn("ID");
         subsmodel.addColumn("Term");
	subsmodel.addColumn("Goal");
	createGui();
     }
     
     public void createGui() {
         messages = new JTextArea(20, 60);
         messages.setEditable(false);
         JScrollPane msgsscrollPane = 
             new JScrollPane(messages,
                             JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
                             JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
         
         JLabel msglabel = new JLabel("Messages: ");
         
         subscriptions = new JTable(subsmodel);
         subscriptions.setPreferredScrollableViewportSize(new Dimension(500, 300));
         subscriptions.setFillsViewportHeight(true);
         ListSelectionModel listSelectionModel = subscriptions.getSelectionModel();
         
         listSelectionModel.addListSelectionListener(new SelectionHandler());
        
         JScrollPane subsscrollPane = 
             new JScrollPane(subscriptions,
                             JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
                             JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
         
         JLabel subslabel = new JLabel("Subscriptions: ");
         
         JLabel subterm = new JLabel("Subscription Term: ");
         subtermtext = new JTextField(20);
         
         JLabel subgoal = new JLabel("Subscription Goal: ");
         subgoaltext = new JTextField(20);
         
         subsubmit = new JButton("Submit");
         subsubmit.addActionListener(this);
         subunsubmit = new JButton("Unsubmit Selected");
         subunsubmit.addActionListener(this);
         
         JLabel notification = new JLabel("Notification: ");
         notifytext = new JTextField(20);
         notifysend = new JButton("Send");
         notifysend.addActionListener(this);
         
         JLabel register = new JLabel("Register Name: ");
         registertext = new JTextField(20);
         register_button = new JButton("Register");
         unregister_button = new JButton("Unregister");
         register_button.addActionListener(this);
         unregister_button.addActionListener(this);
         
         JLabel p2pmsg = new JLabel("P2P Message: ");
         p2pmsgtext = new JTextField(20);
         JLabel p2paddr = new JLabel("P2P Address: ");
         p2paddrtext = new JTextField(20);
         p2psend_button = new JButton("Send");
         p2psend_button.addActionListener(this);
         
         
         GroupLayout layout = new GroupLayout(getContentPane());
         getContentPane().setLayout(layout);
         layout.setAutoCreateGaps(true);
         layout.setAutoCreateContainerGaps(true);
         
         layout.setHorizontalGroup(layout.createSequentialGroup()
            .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
                  .addComponent(msglabel)
		      .addComponent(msgsscrollPane)
		      .addComponent(subslabel)
		      .addComponent(subsscrollPane)
		      .addGroup(layout.createSequentialGroup()
				.addComponent(subterm)
				.addComponent(subtermtext)
				.addComponent(subgoal)
				.addComponent(subgoaltext)
				.addComponent(subsubmit)
				.addComponent(subunsubmit))
		      .addGroup(layout.createSequentialGroup()
				.addComponent(register)
				.addComponent(registertext)
				.addComponent(register_button)
				.addComponent(unregister_button))
		      .addGroup(layout.createSequentialGroup()
				.addComponent(notification)
				.addComponent(notifytext)
				.addComponent(notifysend))
		      .addGroup(layout.createSequentialGroup()
				.addComponent(p2pmsg)
				.addComponent(p2pmsgtext)
				.addComponent(p2paddr)
				.addComponent(p2paddrtext)
				.addComponent(p2psend_button))));


	layout.setVerticalGroup(layout.createSequentialGroup()
	    .addComponent(msglabel)
	    .addComponent(msgsscrollPane)
	    .addComponent(subslabel)
	    .addComponent(subsscrollPane)
	    .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
		      .addComponent(subterm)
		      .addComponent(subtermtext)
		      .addComponent(subgoal)
		      .addComponent(subgoaltext)
		      .addComponent(subsubmit)
		      .addComponent(subunsubmit))
	    .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
		      .addComponent(register)
		      .addComponent(registertext)
		      .addComponent(register_button)
		      .addComponent(unregister_button))
	    .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
		      .addComponent(notification)
		      .addComponent(notifytext)
		      .addComponent(notifysend))
	    .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
		      .addComponent(p2pmsg)
		      .addComponent(p2pmsgtext)
		      .addComponent(p2paddr)
		      .addComponent(p2paddrtext)
		      .addComponent(p2psend_button)));
        
	layout.linkSize(SwingConstants.VERTICAL, subtermtext, subsubmit);
	layout.linkSize(SwingConstants.VERTICAL, subgoaltext, subsubmit);
	layout.linkSize(SwingConstants.VERTICAL, notifytext, notifysend);
        
        
        
	PedroListener msgListener = new MsgReady();
	client.addPedroListener(msgListener);
        
    }
     
     
     private class MsgReady implements PedroListener {
         
         public void pedroReceived(PedroEvent event) {
             while (client.notification_ready()) {
                 String msg = client.get_notification();
                 int index = msg.indexOf(' ');
                 messages.append(msg.substring(index+1));
             }
         }
     }
     
     class SelectionHandler implements ListSelectionListener {
         public void valueChanged(ListSelectionEvent e) { 
             ListSelectionModel lsm = (ListSelectionModel)e.getSource();
             selected = lsm.getMaxSelectionIndex();
         }
     }
     
     
     
     public void actionPerformed(ActionEvent evt) {
         if (evt.getSource() == subsubmit) {
             String term = subtermtext.getText();
             String goal = subgoaltext.getText();
             int id = 0;
	     id = client.subscribe(term, goal);
             if (id > 0) {
                 subsmodel.addRow(new Object[]{id, term, goal});
             }
         }
         if (evt.getSource() == subunsubmit && selected != -1) {
             Integer id = (Integer)((Vector)subsmodel.getDataVector().elementAt(selected)).elementAt(0);
             client.unsubscribe(id);
             subsmodel.removeRow(selected);
             selected = -1;
         }
         
         if (evt.getSource() == notifysend) {
             String notify = notifytext.getText();
             client.notify(notify);
         }
         if (evt.getSource() == register_button) {
             name = registertext.getText();
             if (client.register(name) != 0) registered = true;
         }
         if (evt.getSource() == unregister_button && registered) {
             client.deregister(name);
             registered = false;
             registertext.setText("");
         }
         if (evt.getSource() == p2psend_button && registered) {
             String msg = p2pmsgtext.getText();
             String addr = p2paddrtext.getText();
             client.p2p(addr, msg);
         }
         
     }
     

    public static void main(String[] args) {
	String machine = "";
	int i = 0;
	while (i < args.length) {
	    if (args[i].equals("-M")) {
		machine = args[i+1];
		break;
	    }
	    i++;
	}
	PedroGUI pedrogui = new PedroGUI(machine);
		
	pedrogui.pack();
	pedrogui.addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent e) {
		    System.exit(0);
		}
	    });
	pedrogui.setVisible(true);
    }

}