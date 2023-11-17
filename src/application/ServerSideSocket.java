
package application; 

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket; 
import javax.inject.Inject; 
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication; 
import com.kuka.roboticsAPI.controllerModel.Controller;

import java.net.*;
import java.io.*;


public class ServerSideSocket extends RoboticsAPIApplication {

	 public void run() {
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
