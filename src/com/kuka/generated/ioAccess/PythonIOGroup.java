package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Python</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * Schnittstelle zu python skript

 */
@Singleton
public class PythonIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Python'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Python'
	 */
	@Inject
	public PythonIOGroup(Controller controller)
	{
		super(controller, "Python");

		addInput("Offset_Teil_mm_Y_Richtung", IOTypes.INTEGER, 16);
		addInput("Winkel_Teil_degree", IOTypes.INTEGER, 16);
		addInput("Geschwindigkeit_Foerderband", IOTypes.INTEGER, 16);
		addInput("Offset_Teil_mm_X_Richtung", IOTypes.INTEGER, 16);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Offset_Teil_mm_Y_Richtung</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'Offset_Teil_mm_Y_Richtung'
	 */
	public java.lang.Integer getOffset_Teil_mm_Y_Richtung()
	{
		return getNumberIOValue("Offset_Teil_mm_Y_Richtung", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Winkel_Teil_degree</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'Winkel_Teil_degree'
	 */
	public java.lang.Integer getWinkel_Teil_degree()
	{
		return getNumberIOValue("Winkel_Teil_degree", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Geschwindigkeit_Foerderband</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'Geschwindigkeit_Foerderband'
	 */
	public java.lang.Integer getGeschwindigkeit_Foerderband()
	{
		return getNumberIOValue("Geschwindigkeit_Foerderband", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Offset_Teil_mm_X_Richtung</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'Offset_Teil_mm_X_Richtung'
	 */
	public java.lang.Integer getOffset_Teil_mm_X_Richtung()
	{
		return getNumberIOValue("Offset_Teil_mm_X_Richtung", false).intValue();
	}

}
