package fri_masterthesis_mirko;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import fastRobot_ROS2_HUMBLE.AngleConverter;

/**
 * Creates a FRI Session.
 */
public class FRI extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private Tool TCP;
	double speed;
	int safePos;
	
 // hinzufügen einer profinet io zum abschalten der fahrfreigabe @todo  
	
    PositionControlMode ctrMode = new PositionControlMode();
    PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
    FRIJointOverlay jointOverlay;
    
	static double[] degrees = {
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0
		};
    static double[] radians = AngleConverter.degreesToRadians(degrees);

	private static final JointPosition INITIAL_POSITION = new JointPosition(
            radians[0], radians[1], radians[2],
            radians[3], radians[4], radians[5],
            radians[6]
        );
	
    IFRISessionListener listener = new IFRISessionListener(){
    	@Override
    	public void onFRIConnectionQualityChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("QualityChangedEvent - quality:" +
    	friChannelInformation.getQuality()+"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	@Override
    	public void onFRISessionStateChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("SessionStateChangedEvent - session state:" +
    	friChannelInformation.getFRISessionState() +"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	};
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.0.21";
        
		TCP = getApplicationData().createFromTemplate("Lego_Sauger");

        
		// Inizialisieren der Geschwindigkeiten bei PTP Bewegungen
		speed = 1;
		
		// Inizialisieren der SafePos-Hï¿½he !!!Muss 100 mm bleiben!!!
		safePos = 100;

    }

    @Override
    public void run()
    {

        TCP.attachTo(_lbr.getFlange());
			
		// Lineare Fahrt Senkrecht nach oben um 200 mm 
		//getLogger().info("Lineare Fahrt Senkrecht nach oben um 200 mm");
		//TCP.move(linRel(Transformation.ofDeg(0,0,-1*safePos,0,0,0),getApplicationData().getFrame("/A_Lego_Base/E1")).setJointVelocityRel(0.1));
		//_medflange.setLEDRed(true);
    	double speed_init = 0.7;
		getLogger().info("Init POS PTP");
		_lbr.move(ptp(INITIAL_POSITION).setJointVelocityRel(speed_init));

        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(5);
        //friConfiguration.registerIO() TODO
        getLogger().info("Creating FRI connection to "  + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

       
		PositionControlMode ctrMode = new PositionControlMode();
		posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.POSITION);
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
        getLogger().info("FRI connection established.");

        // move to start pose
		getLogger().info("Init POS PTP");

        //_lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));

		getLogger().info("Lets Go Position Mode");

        //_lbr.move(posHold.addMotionOverlay(jointOverlay));
        _lbr.getCurrentJointPosition();
        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRI app = new FRI();
        app.runApplication();
    }

}
