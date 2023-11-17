package application;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.net.*;
import java.io.*;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class mma_22_vordergrund extends RoboticsAPIApplication {
	@Override
	public void initialize() {
		// initialize your application here
	}
	
	 public void run() {
			//System.out.println("Kuka ist kacka"); 
			try {
				int serverPort = 30001;
				ServerSocket serverSocket = new ServerSocket(serverPort);
				serverSocket.setSoTimeout(5000000); 
				while(true) {
					System.out.println("Waiting for client on port " + serverSocket.getLocalPort() + "..."); 

					Socket server = serverSocket.accept();
					System.out.println("Just connected to " + server.getRemoteSocketAddress()); 
					PrintWriter toClient = 
						new PrintWriter(server.getOutputStream(),true);
					BufferedReader fromClient =
						new BufferedReader(
								new InputStreamReader(server.getInputStream()));
					

					String line = fromClient.readLine();

					
					System.out.println("Server received: " + line); 
					toClient.println("Thank you for connecting to " + server.getLocalSocketAddress() + "\nGoodbye!"); 
				}
			}
			catch(UnknownHostException ex) {
				ex.printStackTrace();
			}
			catch(IOException e){
				e.printStackTrace();
			}
		  }
			
		  public static void main(String[] args) {
				ServerSideSocket srv = new ServerSideSocket();
				srv.run();
		  }
		}