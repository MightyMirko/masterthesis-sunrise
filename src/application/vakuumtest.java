package application;


import javax.inject.Inject;

import com.kuka.generated.ioAccess.VakuumIOGroup;
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
public class vakuumtest extends RoboticsAPIApplication {
	@Inject
	private VakuumIOGroup CVakuum;
	
	@Override
	public void initialize() {
		// initialize your application here
	}

	@Override
	public void run() {
		// your application execution starts here
		CVakuum.setVakuumON(false);
		try {
			
			for (int i = 0; i < 2; i++){
				CVakuum.setVakuumON(true);
				Thread.sleep(10);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(60);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(10);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(60);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(10);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(60);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(300);
				
				CVakuum.setVakuumON(true);
				Thread.sleep(100);
				
				CVakuum.setVakuumON(false);
				Thread.sleep(60);
			}
			
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}