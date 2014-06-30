package com.magnin.imuaccess;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.InetAddress;

import android.app.Activity;
//import android.content.Context;
//import android.widget.ToggleButton;


public class TCPClient implements Runnable {

	private String IP = null;
	private int port;
	private OnMessageReceived serverMsgHandler = null;
	public OnSendingMessage msgSender = null;
	public boolean running = true;
	public PrintWriter out = null;
    private BufferedReader in = null;
    Socket socket = null;
    
    Activity superContext = null;

	public TCPClient (String IP, int port, OnMessageReceived serverMsgHandler, OnSendingMessage msgSender) {
		this.IP = IP;
		this.port = port;
		this.serverMsgHandler = serverMsgHandler;
		this.msgSender = msgSender;
		System.out.println("TCPClient created");
	}
	public TCPClient (String IP, int port, OnMessageReceived serverMsgHandler, OnSendingMessage msgSender, Activity superContext) {
		this.IP = IP;
		this.port = port;
		this.serverMsgHandler = serverMsgHandler;
		this.msgSender = msgSender;
		this.superContext = superContext;
		System.out.println("TCPClient created");
	}
	
	private boolean connectWithServer() {
		try {
			socket = null;
			System.out.println("Trying to connect:");
			System.out.println(this.IP);
			System.out.println(this.port);
			System.out.println("...");

			socket = new Socket(InetAddress.getByName(this.IP), this.port);
            out = new PrintWriter(socket.getOutputStream());
            in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            
            return true;
        } catch (IOException e) {
        	System.err.println("Connection fail:");
            e.printStackTrace();
            return false;
        }
    }
	
	private void disConnectFromServer() {
        if ( socket != null && socket.isConnected() ) {
            try {
                in.close();
                out.close();
                socket.close();
                System.out.println("Disconnected!");
            } catch (Exception e) {
            	System.err.println("Disconnection fail:");
                e.printStackTrace();
            }
        }
    }
	
	public void run() {
		while (running) {
			while (running && !connectWithServer()); // loop until be connected
			if (running) System.out.println("Connected!");
			try {
				while (running) {
					String serverMessage = in.readLine();
					if ( serverMessage != null ) {
						//call the method messageReceived implemented in MainActivity class
						//System.out.println("Server msg:");
						//System.out.println(serverMessage);
						serverMsgHandler.messageReceived(serverMessage);
						serverMessage = null;
					}
				}
			} catch (Exception e) {
				System.err.println("Cannot read msg:");
				e.printStackTrace();
			} finally {
				//the socket must be closed. It is not possible to reconnect to this socket
				// after it is closed, which means a new socket instance has to be created.
				disConnectFromServer();
			}
		}
	}
	
	public void stop() {
		System.out.println("stop() was called");
		running = false;
		//disConnectFromServer(); //make sure to disconnect (maybe it is already disconnect)
    }
	
	//Declare the interface. The method messageReceived(String message) will/must be implemented in the MyActivity
    //class on the TCPClient instantiation
	public interface OnSendingMessage
	{
		public void sendMessage (String message);
	}
	
	//Declare the interface. The method messageReceived(String message) will/must be implemented in the MyActivity
    //class on the TCPClient instantiation
    public interface OnMessageReceived
	{
        public void messageReceived (String message);
    }
	

}
