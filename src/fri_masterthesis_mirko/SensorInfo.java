package fri_masterthesis_mirko;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation.FRISessionState;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
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
import com.kuka.task.ITaskLogger;

import fastRobot_ROS2_HUMBLE.AngleConverter;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class SensorInfo extends RoboticsAPICyclicBackgroundTask {
	@Inject
	private ITaskLogger logger;
	@Inject
    private Controller _lbrController;
    private String _clientName;
	@Inject
	private LBR _lbr;
	FRIChannelInformation friChannelInformation;
    IFRISessionListener listener = new IFRISessionListener(){
    	@Override
    	public void onFRIConnectionQualityChanged(
    	FRIChannelInformation friChannelInformation){
    		logger.info("QualityChangedEvent - quality:" +
    	friChannelInformation.getQuality()+"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	@Override
    	public void onFRISessionStateChanged(
    	FRIChannelInformation friChannelInformation){
    		logger.info("SessionStateChangedEvent - session state:" +
    	friChannelInformation.getFRISessionState() +"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	};
    
	@Override
	public void initialize() {
		// initialize your task here
		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);

				_lbrController = (Controller) getContext().getControllers().toArray()[0];
				// **********************************************************************
				// *** change next line to the FRIClient's IP address                 ***
				// **********************************************************************
				_clientName = "172.31.0.21";

	}

	@Override
	public void runCyclic() {
		boolean tmp = true;
		
		
// your task execution starts here
	    FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
    	friConfiguration.setSendPeriodMilliSec(5);
    	
		FRISession friSession = new FRISession(friConfiguration);
 //logger.info("Creating FRI connection to "  + friConfiguration.getHostName());
 //logger.info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
    //             + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

         try{
        	 friSession.await(10, TimeUnit.SECONDS);
         }
         catch (final TimeoutException e)
         {
        	 logger.error(e.getLocalizedMessage());
             //friSession.close();
 			tmp = false;
             return;
         }

         //logger.info("FRI connection established.");	
	
         //logger.info(friChannelInformation.toString());
		while (tmp){
			
			if (friChannelInformation.getFRISessionState() == FRISessionState.IDLE)
				tmp = false;			

		}
		friSession.close();
	}
}