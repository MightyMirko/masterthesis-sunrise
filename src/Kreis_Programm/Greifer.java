// Einbinden des Greifers mit Info-Meldungen

package Kreis_Programm; // Name des Pakets in der die Roboterapplikation liegt

import javax.inject.Inject;

import com.kuka.roboticsAPI.geometricModel.Tool; // Basisklasse fuer Werkzeuge
import com.kuka.task.ITaskLogger; // Anzeigen von Ausgaben auf smartHMI
import com.kuka.generated.ioAccess.GripperIOGroup;

public class Greifer extends Tool {
	@Inject
	private ITaskLogger logger;
	
	@Inject
	private GripperIOGroup io;

	public Greifer(String name) { // Standardkonstruktor der Klasse Greifer
		super(name);
	}
// test kommentar
	
// Methode openGripper() zum Oeffnen des Greifers
public void openGripper(){
	logger.info ("Greifer öffnen");
	io.setSetClose(false);
	io.setSetOpen(true);
};

// Methode closeGripper() zum Schließen des Greifers
public void closeGripper(){
	logger.info ("Greifer schließen");
	io.setSetOpen(false);
	io.setSetClose(true);
};

}


// PDF Seite 455
