package application;

import javax.inject.Inject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;



import com.kuka.generated.ioAccess.VakuumIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
//import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
//import com.kuka.roboticsAPI.motionModel.PTP;
//import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class AutoPlanBau_test extends RoboticsAPIApplication {
	@Inject
	private LBR lbr;
	private Tool TCP;
	private final static String informationTextvierer=
			"Vierersteine nachfüllen und mit OK bestätigen!";
	
	private final static String informationTextachter=
			"Achtersteine nachfüllen und mit OK bestätigen!";
	@Inject
	private VakuumIOGroup CVakuum;

	int gotData = 0;
	String line = "leer";
	
	
	int blendingCart;
	int blendingCartaway;
	int blendingCart_Safepos;
	double speed;
	
	int safePos;
	int impendance_distance_hol;
	int impendance_distance_vhol;
	int impendance_distance_abl;
	int impendance_distance_vabl;
	int PalAbsx;
	int PalAbsy;
	
	int Zaehler8;
	int Zaehler4;
	
	double BSB;
	double BSH;
	
	private static final int stiffnessZ = 5000;
	private static final int stiffnessY = 3000;
	private static final int stiffnessX = 3000;
			
	
	double[] positionenx;
	double[] positioneny;
	double[] positionenz;
	int[] rotation;
	int[] Stein;

	double[] BSList;
//	List BSList = new ArrayList<Double>();
	
	int BSListlen;
	String Bauplanname = "";
	

	@Override
	public void initialize() {
		
		// Initialisieren des TCP
		TCP = getApplicationData().createFromTemplate("Lego_Sauger");

		// Inizialisieren der Verschleifradien im Kartesischen System
		blendingCart = 100; 
		blendingCartaway = 150;
		blendingCart_Safepos = 800;
		
		// Inizialisieren der Geschwindigkeiten bei PTP Bewegungen
		speed = 1;
		
		// Inizialisieren der SafePos-Höhe !!!Muss 100 mm bleiben!!!
		safePos = 100;
		
		// Inizialisieren der Impendance Distanz 
		impendance_distance_vhol = 2;
		impendance_distance_vabl = 2;
		impendance_distance_hol = 6*impendance_distance_vhol;
		impendance_distance_abl = 12*impendance_distance_vabl;
		
		// Initialisieren der Bausteinbreite und Bausteinhöhe
		BSB = 32.065;
		BSH = 19.1;;
		
		// Initialisieren der Palettenzähler --> Palette ist Voll
		Zaehler8 = 0;
		Zaehler4 = 0;
		
		// Abstand in der Palette in x und y Richtung
		PalAbsx = 63;
		PalAbsy = 90;
		
		// Erstellen einer Pufferliste
		BSListlen = 800;
		BSList = new double[BSListlen];
		
		// TCP Socket Verbindung
		
				try {
					int serverPort = 30001;
					ServerSocket serverSocket = new ServerSocket(serverPort);
					serverSocket.setSoTimeout(10000); 
					while(true) {
						System.out.println("Waiting for client on port " + serverSocket.getLocalPort() + "..."); 

						Socket server = serverSocket.accept();
						System.out.println("Just connected to " + server.getRemoteSocketAddress()); 
						gotData = 1;
						PrintWriter toClient = 
							new PrintWriter(server.getOutputStream(),true);
						BufferedReader fromClient =
							new BufferedReader(
									new InputStreamReader(server.getInputStream()));
						

						line = fromClient.readLine();

						
						System.out.println("Server received: " + line); 
						toClient.println("Thank you for connecting to " + server.getLocalSocketAddress() + "\nGoodbye!"); 
						
						if (gotData == 1){
							break;
						}
						
						
					}
				}
				catch(UnknownHostException ex) {
					ex.printStackTrace();
				}
				catch(IOException e){
					e.printStackTrace();
				}
			  
		
////		 // String vereinzeln an "," und schreibe in string result
////        List BSList = new ArrayList<Double>();
//        String Bauplanname = "";
//        String[] string_result = "Quadrat, 8.0, 90.0, 3.5, 5.0, 0.0, 8.0, 90.0, 3.5, 7.0, 0.0, 8.0, 0.0, 4.0, 8.5, 0.0, 8.0, 0.0, 6.0, 8.5, 0.0, 8.0, 90.0, 7.5, 8.0, 0.0, 8.0, 90.0, 7.5, 6.0, 0.0, 8.0, 0.0, 7.0, 4.5, 0.0, 8.0, 0.0, 5.0, 4.5, 0.0, 8.0, 0.0, 4.0, 4.5, 1.0, 8.0, 90.0, 3.5, 6.0, 1.0, 8.0, 90.0, 3.5, 8.0, 1.0, 8.0, 0.0, 5.0, 8.5, 1.0, 8.0, 0.0, 7.0, 8.5, 1.0, 8.0, 90.0, 7.5, 7.0, 1.0, 8.0, 90.0, 7.5, 5.0, 1.0, 8.0, 0.0, 6.0, 4.5, 1.0, 8.0, 90.0, 3.5, 5.0, 2.0, 8.0, 90.0, 3.5, 7.0, 2.0, 8.0, 0.0, 4.0, 8.5, 2.0, 8.0, 0.0, 6.0, 8.5, 2.0, 8.0, 90.0, 7.5, 8.0, 2.0, 8.0, 90.0, 7.5, 6.0, 2.0, 8.0, 0.0, 7.0, 4.5, 2.0, 8.0, 0.0, 5.0, 4.5, 2.0, 8.0, 0.0, 4.0, 4.5, 3.0, 8.0, 90.0, 3.5, 6.0, 3.0, 8.0, 90.0, 3.5, 8.0, 3.0, 8.0, 0.0, 5.0, 8.5, 3.0, 8.0, 0.0, 7.0, 8.5, 3.0, 8.0, 90.0, 7.5, 7.0, 3.0, 8.0, 90.0, 7.5, 5.0, 3.0, 8.0, 0.0, 6.0, 4.5, 3.0".split(",");
//

				String[] string_result = line.split(",");
				
        for (int x = 0; x < string_result.length; x++) {

            if (x == 0) {
                Bauplanname = string_result[x];
            } else {
            	//System.out.println(x);
                double zahl = Double.parseDouble(string_result[x]);
                BSList[x] = zahl;
                //System.out.println(zahl);
                
            }
        }
        
        System.out.println(BSList);
		
	}

	@Override
	public void run() {
		
		
		
		
		// Inizialisieren der Impendance Parameter
		getLogger().info("Initialisieren der Impendance-Parameter");
		CartesianImpedanceControlMode impedanceControlMode;
		impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
	
		// Vakuum als default wert nach starten des Programms ausschalten
//		if (CVakuum.getVakuumON() == true){
//			CVakuum.setVakuumON(false);
//			getLogger().info("Setze Output auf False");
//		}

		
		// Zuweisung des TCP an den Roboterflange
		TCP.attachTo(lbr.getFlange());
		
		
		// Lineare Fahrt Senkrecht nach oben um 200 mm 
		getLogger().info("Lineare Fahrt Senkrecht nach oben um 200 mm");
		TCP.move(linRel(Transformation.ofDeg(0,0,-1*safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.1));
		
		// For-Schleife über die Länge der Liste der Bausteine 
		for (int i = 0; (i < BSListlen); i = i+5){
			
			// Beender der FOR-Schleife, wenn kein Zahlenwert mehr ür die bausteinart ankommt
			if (BSList[i+1] == 0){
				break;
			}
			
			// Anfahren und Verschleifen der SafePos zwischen Palette und der Ablage
			getLogger().info("Anfahren und Verschleifen der SafePos zwischen Palette und der Ablage");
			TCP.moveAsync(ptp(getApplicationData().getFrame("/A_Lego_SavePos")).setBlendingCart(blendingCart_Safepos).setJointVelocityRel(0.5));
			
			// if else if Entscheidung ob 4er oder 8er Stein
			getLogger().info("Baustein holen");
			if ((BSList[i+1] == 4.0) & (Zaehler4<=7)){
					
					// Vierer holen
					getLogger().info("Vierer holen");
					
					//  Relative Bewegung zu dem nächsten 4er Baustein
					ObjectFrame vPosviererObjectFrame = getApplicationData().getFrame("/A_Lego_Pal_test/Lego");
					Frame vPosviererFrame = vPosviererObjectFrame.copyWithRedundancy(vPosviererObjectFrame);
					Transformation vPosvierertrafo = Transformation.ofDeg(PalAbsx*Zaehler4, 0,-safePos, 0, 0, 0); 
					vPosviererFrame.transform(vPosvierertrafo);
					TCP.move(ptp(vPosviererFrame).setBlendingCart(blendingCart).setJointVelocityRel(speed));
					
					//Relative Bewegung auf die Bausteinposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war
					getLogger().info("Relative Bewegung auf die Bausteinposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war");
					TCP.move(linRel(Transformation.ofDeg(0,0,safePos-impendance_distance_vhol,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.3));
					
//					
//					// Anschalten des Vakuums
//					getLogger().info("Anschalten des Vakuums");
//					CVakuum.setVakuumON(true);
//					
					// Relative Bewegung in den Baustein hinein mit Impendanz Modus --> Erzeugte Federkraft ist 3+1* Federkonstante
					getLogger().info("Relative Bewegung in den Baustein hinein mit Impendanz Modus");
					TCP.move(linRel(Transformation.ofDeg(0,0,impendance_distance_hol,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.1).setMode(impedanceControlMode));
					
					// Warten
					getLogger().info("Warten 1500 ms");
					try {
						Thread.sleep(1500);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					
					// Relative Bewegung um 100 mm on der Fügeposition nach oben
					getLogger().info("Relative Bewegung um 100 mm on der Fügeposition nach oben");
					TCP.move(linRel(Transformation.ofDeg(0,0,-safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.1).setMode(impedanceControlMode));	
					
					// Zählerwert um 1 erhöhen
					getLogger().info("Zählerwert um 1 erhöhen");
					Zaehler4 = Zaehler4+1;			
			}
			
			else if ((BSList[i+1] == 8.0)& (Zaehler8<=7)){		
					
					// Achter holen
					getLogger().info("Achter holen");
				
					//  Relative Bewegung zu dem nächsten 4er Baustein
					ObjectFrame vPosachterObjectFrame = getApplicationData().getFrame("/A_Lego_Pal_test/Lego");
					Frame vPosachterFrame = vPosachterObjectFrame.copyWithRedundancy(vPosachterObjectFrame);
					Transformation vPosvierertrafo = Transformation.ofDeg(PalAbsx*Zaehler8, -(PalAbsy),-safePos, 0, 0, 0); 
					vPosachterFrame.transform(vPosvierertrafo);
					TCP.move(ptp(vPosachterFrame).setBlendingCart(blendingCart).setJointVelocityRel(speed));
					
					// Relative Bewegung auf die Bausteinposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war
					getLogger().info("Relative Bewegung auf die Bausteinposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war");
					TCP.move(linRel(Transformation.ofDeg(0,0,safePos-impendance_distance_vhol,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.3));
//					
//					// Anschalten des Vakuums
//					getLogger().info("Anschalten des Vakuums");
//					CVakuum.setVakuumON(true);
					
					// Relative Bewegung in den Baustein hinein mit Impendanz Modus --> Erzeugte Federkraft ist 3+1* Federkonstante
					getLogger().info("Relative Bewegung in den Baustein hinein mit Impendanz Modus");
					TCP.move(linRel(Transformation.ofDeg(0,0,impendance_distance_hol,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.1).setMode(impedanceControlMode));
					
					// Warten
					getLogger().info("Warten 1500 ms");
					try {
						Thread.sleep(1500);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					
					// Relative Bewegung um 100 mm on der Abholposition nach oben
					TCP.move(linRel(Transformation.ofDeg(0,0,-safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Pal_test/Lego")).setJointVelocityRel(0.1).setMode(impedanceControlMode));	
					getLogger().info("Relative Bewegung um 100 mm on der Fügeposition nach oben");
					
					// Zählerwert um 1 erhöhen
					getLogger().info("Zählerwert um 1 erhöhen");
					Zaehler8 = Zaehler8+1;
			}
			
			// Anfahren und Verschleifen der SafePos zwischen Palette und der Ablage
			getLogger().info("Anfahren und Verschleifen der SafePos zwischen Palette und der Ablage");
			TCP.moveAsync(ptp(getApplicationData().getFrame("/A_Lego_SavePos")).setBlendingCart(blendingCart_Safepos).setJointVelocityRel(speed));
			
			// if Abfragen ob die Paletten nochBausteine einthalten --> wenn NEIN, dann wird eine Meldung ausgegeben, dass diese nachgefüllt werden sollen.
			if (Zaehler4 == 7){
				getLogger().info("Show modal dialog and wait for user to confirm");
		        
				// Benutzerabfrage ob die Palette wieder befüllt wurde
				int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationTextvierer, "OK", "Cancel");
		        if (isCancel == 1)
		        {
		            return;
		        }
		        
		        // Zähler Rücksetzen
		        getLogger().info("Vierer Zähler zurrückgesetzt");
				Zaehler4 = 0;
			}
			
			// if Abfragen ob die Paletten nochBausteine einthalten --> wenn NEIN, dann wird eine Meldung ausgegeben, dass diese nachgefüllt werden sollen.
			if (Zaehler8 == 7){
				getLogger().info("Show modal dialog and wait for user to confirm");
				
				// Benutzerabfrage ob die Palette wieder befüllt wurde
		        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationTextachter, "OK", "Cancel");
		        if (isCancel == 1)
		        {
		            return;
		        }
		     
		        
		        // Zähler Rücksetzen
		        getLogger().info("Achter Zähler zurrückgesetzt");
				Zaehler8 = 0;
			}
			// Stein Nummer
			getLogger().info("SteinNummmer"+i);
	
			// Ablegen des bausteins auf Variable Positionen
			
			
			//  Relative Bewegung zu dem nächsten 4er Baustein
			ObjectFrame vPosAblObjectFrame = getApplicationData().getFrame("/A_Lego_Base/E1");
			Frame vPosAblFrame = vPosAblObjectFrame.copyWithRedundancy(vPosAblObjectFrame);
			Transformation vPosAbltrafo = Transformation.ofDeg(BSB*(BSList[i+3]),-(BSB*(BSList[i+4])+0.4),-safePos, 90-BSList[i+2]-2, 0, 0); 
			vPosAblFrame.transform(vPosAbltrafo);
			TCP.move(ptp(vPosAblFrame).setBlendingCart(blendingCart).setJointVelocityRel(speed));
			
			// Relative Bewegung auf die Ablageposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war
			getLogger().info("Relative Bewegung auf die Ablageposition Abzüglich der Distanz die für die ImpendanzBewegug vorgesehen war");
			TCP.move(linRel(Transformation.ofDeg(0,0,(safePos-(BSList[i+5]*BSH)-impendance_distance_vabl),0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.3));
			
			// Relative Bewegung in die Ablage hinein mit Impendanz Modus --> Erzeugte Federkraft ist 3+1* Federkonstante
			getLogger().info("Relative Bewegung in die Ablage hinein mit Impendanz Modus");
			TCP.move(linRel(Transformation.ofDeg(0,0,(impendance_distance_abl),0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.1).setMode(impedanceControlMode));
//			
//			// Abschalten des Vakuums
//			getLogger().info("Abschalten des Vakuums");
//			try {
//				CVakuum.setVakuumON(false);
//				
//				// Warten
//				Thread.sleep(1500);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
			
			// Relative Bewegung um 100 mm on der Fügeposition nach oben
			getLogger().info("Relative Bewegung um 100 mm on der Fügeposition nach oben");
			TCP.move(linRel(Transformation.ofDeg(0,0,-safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.3).setMode(impedanceControlMode));	
			
			// i Zähler hochzählen
			//i = i+4;
		
		}
		
		// Relative Bewegung um 100 mm on der Fügeposition nach oben
		getLogger().info("Relative Bewegung um 100 mm on der Fügeposition nach oben");
		TCP.move(linRel(Transformation.ofDeg(0,0,-safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.3));

	}
	
}
	
	
	
	
	
	
	
	
