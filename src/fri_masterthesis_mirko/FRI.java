package fri_masterthesis_mirko;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseRequestService;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSR;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.connectionLib.Message;
import com.kuka.roboticsAPI.controllerModel.sunrise.positionMastering.PositionMastering;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.requestModel.GetJointLimitRequest;

import fastRobot_ROS2_HUMBLE.AngleConverter;
/**
 * C:reates a FRI Sesaeeesion.
 */ 
public class FRI extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    //private String _clientName;
    private Tool virtualGripper;
	double speed;
	int safePos;
	//private HandGuidingMotion motion;
	
 // hinzuf�gen einer profinet io zum abschalten der fahrfreigabe @todo  
	
    /*PositionControlMode ctrMode = new PositionControlMode();
    PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
    FRIJointOverlay jointOverlay;
*/
    private final static double sideOffset = Math.toRadians(5);       // offset in radians for side motion
    private double joggingVelocity = 0.7;                            // relative velocity
    private final static int axisId[] = {0, 1, 2, 3, 4, 5, 6};        // axes to be referenced
    private final static int GMS_REFERENCING_COMMAND = 2;             // safety command for GMS referencing
    private final static int COMMAND_SUCCESSFUL = 1;
    private int positionCounter = 0;
    
    
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
	
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        //_clientName = "172.31.0.21";
        virtualGripper = getApplicationData().createFromTemplate("myVirtualGripper");
        virtualGripper.attachTo(_lbr.getFlange());
        
        
		// Initialisieren der Geschwindigkeiten bei PTP Bewegungen
		speed = 1;
		
		// Initialisieren der SafePos-Höhe !!!Muss 100 mm bleiben!!!
		safePos = 100;

    }

    @Override
    public void run()
    {

    	double speed_init = 0.7;
		getLogger().info("Init POS PTP");
		_lbr.move(ptp(INITIAL_POSITION).setJointVelocityRel(speed_init));

		//virtualGripper.move(ptp(getApplicationData().getFrame("/A_virtualGripHome")));
		getLogger().info("Lets Go Position Referencing Mode");
		boolean referenced = true;
		boolean repeat = true; 		
		while (repeat) {
		    
            simulateSingleJointMotion(1, Math.toRadians(-30)); // Joint 3 (0-based index) to target angle 45 degrees
            simulateSingleJointMotion(2, Math.toRadians(-45)); // Joint 3 (0-based index) to target angle 45 degrees
            
	        getLogger().info("Moving first joint from -130 to 170 degrees");
	        _lbr.move(moveFirstJointAsync(-130));
	        _lbr.move(moveFirstJointAsync(130));
	        
	        simulateSingleJointMotion(2, Math.toRadians(45)); // Joint 3 (0-based index) to target angle 45 degrees
	        simulateSingleJointMotion(1, Math.toRadians(30)); // Joint 3 (0-based index) to target angle 45 degrees
            
			/*repeat = goTest();
	        // Simulate Single Joint Motion for Joint 3
            
	        _lbr.move(move6_async(5, -70));

           _lbr.move(move6_async(5, 70));
            simulateSingleJointMotion(6,Math.toRadians(90));
	        // Simulate Sequential Joint Motion
	        //simulateSequentialJointMotion();
		    */
			if (!referenced){			
			    repeat = goGsm();}
		}

    }
    
    private PTP moveFirstJointAsync(double targetAngle) {
        JointPosition jointPosition = new JointPosition(_lbr.getCurrentJointPosition());
        jointPosition.set(0, Math.toRadians(targetAngle));
        PTP motion = new PTP(jointPosition).setJointVelocityRel(joggingVelocity);
        return motion;
    }

    private PTP move6_async(int axisNo, double targetAngle) {
        JointPosition jointPosition = new JointPosition(_lbr.getCurrentJointPosition());
        jointPosition.set(axisNo, Math.toRadians(targetAngle));
        PTP motion = new PTP(jointPosition).setJointVelocityRel(joggingVelocity);
        return motion;
        }
    
    
    private void simulateSingleJointMotion(int jointIndex, double targetAngle) {
        if (jointIndex < 0 || jointIndex >= 7) {
            getLogger().warn("Invalid joint index.");
            return;
        }

        // Move the specified joint while keeping others stationary
        getLogger().info("Simulating Single Joint Motion - Joint " + (jointIndex + 1) + " to " + Math.toDegrees(targetAngle) + " degrees");
        _lbr.moveAsync(new PTP(getTargetJointPosition(jointIndex, targetAngle)).setJointVelocityRel(joggingVelocity));
        //getLogger().info("End-Effector Velocity: " + _lbr.getExternalForceTorque());
        // Wait for a moment
        try {
            TimeUnit.SECONDS.sleep(1);
        } catch (InterruptedException e) {
            getLogger().error("InterruptedException: " + e.getMessage());
        }
    }

    private void simulateSequentialJointMotion() {
        // Move joints sequentially, one after the other
        getLogger().info("Simulating Sequential Joint Motion");
        for (int i = 0; i < 7; ++i) {
            double targetAngle = Math.toRadians(30 * (i + 1)); // Incremental target angles
            _lbr.move(new PTP(getTargetJointPosition(i, targetAngle)).setJointVelocityRel(joggingVelocity));
            //getLogger().info("End-Effector Velocity: " + _lbr.getExternalForceTorque());

            // Wait for a moment
            try {
                TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
                getLogger().error("InterruptedException: " + e.getMessage());
            }
        }
    }

    private JointPosition getTargetJointPosition(int jointIndex, double targetAngle) {
        JointPosition jointPosition = new JointPosition(_lbr.getJointCount());
        jointPosition.set(jointIndex, targetAngle);
        return jointPosition;
    }
    
    
    
    
    
    
    private boolean goTest()
    {
        performMotion(new JointPosition(
                Math.toRadians(90),
                Math.toRadians(-70),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(40)
                ));
        
        performMotion(new JointPosition(
                Math.toRadians(90),
                Math.toRadians(70),
                Math.toRadians(30),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0)
                ));
        return true;
    }

    private boolean goGsm()
    {
       PositionMastering mastering = new PositionMastering(_lbr);

        boolean allAxesMastered = true;
        for (int i = 0; i < axisId.length; ++i)
        {
            // Check if the axis is mastered - if not, no referencing is possible
            boolean isMastered = mastering.getMasteringInfo("NO_TOOL").getMasteringState(i);
            if (!isMastered)
            {
                getLogger().warn("Axis with axisId " + axisId[i] + " is not mastered, therefore it cannot be referenced");
            }
            
            allAxesMastered &= isMastered;
        }
        
        // We can move faster, if operation mode is T1
        if (OperationMode.T1 == _lbr.getOperationMode())
        {
            joggingVelocity = 0.3;
        }
        else
        {
            joggingVelocity = 0.5;
        }
        
        if (allAxesMastered)
        {
            getLogger().info("Perform position and GMS referencing with 5 positions");
            
            // Move to home position
            getLogger().info("Moving to home position");
            _lbr.move(ptpHome().setJointVelocityRel(joggingVelocity));

            // In this example 5 positions are defined, though each one 
            // will be reached from negative and from positive axis 
            // direction resulting 10 measurements. The safety needs 
            // exactly 10 measurements to perform the referencing.
            performMotion(new JointPosition(Math.toRadians(0.0),
                                            Math.toRadians(16.18),
                                            Math.toRadians(23.04),
                                            Math.toRadians(37.35),
                                            Math.toRadians(-67.93),
                                            Math.toRadians(38.14),
                                            Math.toRadians(-2.13)));
            
            performMotion(new JointPosition(Math.toRadians(18.51),
                                            Math.toRadians(9.08),
                                            Math.toRadians(-1.90),
                                            Math.toRadians(49.58),
                                            Math.toRadians(-2.92),
                                            Math.toRadians(18.60),
                                            Math.toRadians(-31.18)));

            performMotion(new JointPosition(Math.toRadians(-18.53),
                                            Math.toRadians(-25.76),
                                            Math.toRadians(-47.03),
                                            Math.toRadians(-49.55),
                                            Math.toRadians(30.76),
                                            Math.toRadians(-30.73),
                                            Math.toRadians(20.11)));

            performMotion(new JointPosition(Math.toRadians(-48.66),
                                            Math.toRadians(24.68),
                                            Math.toRadians(-11.52),
                                            Math.toRadians(10.48),
                                            Math.toRadians(-11.38),
                                            Math.toRadians(-20.70),
                                            Math.toRadians(20.87)));

            performMotion(new JointPosition(Math.toRadians(9.01),
                                            Math.toRadians(-35.00),
                                            Math.toRadians(24.72),
                                            Math.toRadians(-82.04),
                                            Math.toRadians(14.65),
                                            Math.toRadians(-29.95),
                                            Math.toRadians(1.57)));
            
            // Move to home position at the end
            getLogger().info("Moving to home position");
            _lbr.move(ptpHome().setJointVelocityRel(joggingVelocity));
        }
		return true;
    }

    private void performMotion(JointPosition position)
    {
        getLogger().info("Moving to position #" + (++positionCounter));

        PTP mainMotion = new PTP(position).setJointVelocityRel(joggingVelocity);
        _lbr.move(mainMotion);

        getLogger().info("Moving to current position from negative direction");
        JointPosition position1 = new JointPosition(_lbr.getJointCount());
        for (int i = 0; i < _lbr.getJointCount(); ++i)
        {
            position1.set(i, position.get(i) - sideOffset);
        }
        PTP motion1 = new PTP(position1).setJointVelocityRel(joggingVelocity);
        _lbr.move(motion1);
        _lbr.move(mainMotion);

        // Wait a little to reduce robot vibration after stop.
        ThreadUtil.milliSleep(200);
        
        // Send the command to safety to trigger the measurement
        sendSafetyCommand();

        getLogger().info("Moving to current position from positive direction");
        JointPosition position2 = new JointPosition(_lbr.getJointCount());
        for (int i = 0; i < _lbr.getJointCount(); ++i)
        {
            position2.set(i, position.get(i) + sideOffset);
        }
        PTP motion2 = new PTP(position2).setJointVelocityRel(joggingVelocity);
        _lbr.move(motion2);
        _lbr.move(mainMotion);

        // Wait a little to reduce robot vibration after stop
        ThreadUtil.milliSleep(200);
        
        // Send the command to safety to trigger the measurement
        sendSafetyCommand();
    }


    private void sendSafetyCommand()
    {
        ISunriseRequestService requestService = (ISunriseRequestService) (_lbrController.getRequestService());
        SSR ssr = SSRFactory.createSafetyCommandSSR(GMS_REFERENCING_COMMAND);
        Message response = requestService.sendSynchronousSSR(ssr);
        int result = response.getParamInt(0);
        if (COMMAND_SUCCESSFUL != result)
        {
            getLogger().warn("Command did not execute successfully, response = " + result);
        }
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