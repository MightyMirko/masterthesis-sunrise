package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Vakuum</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * Ansteuern einer Venturiedüse
 */
@Singleton
public class VakuumIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Vakuum'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Vakuum'
	 */
	@Inject
	public VakuumIOGroup(Controller controller)
	{
		super(controller, "Vakuum");

		addDigitalOutput("VakuumON", IOTypes.BOOLEAN, 1);
		addDigitalOutput("VakuumOFF", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital output '<i>VakuumON</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'VakuumON'
	 */
	public boolean getVakuumON()
	{
		return getBooleanIOValue("VakuumON", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>VakuumON</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'VakuumON'
	 */
	public void setVakuumON(java.lang.Boolean value)
	{
		setDigitalOutput("VakuumON", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>VakuumOFF</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'VakuumOFF'
	 */
	public boolean getVakuumOFF()
	{
		return getBooleanIOValue("VakuumOFF", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>VakuumOFF</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'VakuumOFF'
	 */
	public void setVakuumOFF(java.lang.Boolean value)
	{
		setDigitalOutput("VakuumOFF", value);
	}

}
