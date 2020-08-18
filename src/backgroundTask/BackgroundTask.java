package backgroundTask;

import java.util.Timer;
import java.util.TimerTask;

import ioTool.*;

import javax.inject.Inject;

import additionFunction.NutCommThread;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Implementation of a background task.
 * <p>
 * The background task provides a {@link RoboticsAPIBackgroundTask#initialize()}
 * and a {@link RoboticsAPIBackgroundTask#run()} method, which will be called
 * successively in the task lifecycle.<br>
 * The task will terminate automatically after the <code>run</code> method has
 * finished or after stopping the task.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 * 
 * @see UseRoboticsAPIContext
 * 
 */
public class BackgroundTask extends RoboticsAPIBackgroundTask {
	@Inject
	private LBR	lbr;
	
	@Inject
	private Controller		kUKA_Sunrise_Cabinet_1;

	private ExternalIOGroup	exio;
	private ATC_Schunk schunk;
	//private NutRunner nut;
	private Timer tmrEsmCheck;
	
	private final int ESM_NormalMode = 1;
	private final int ESM_EmergencyStop = 2;
	
	public NutCommThread mNutCommThread;
	public Thread mNut_main;
	
	@Override
	public void initialize() {
		// initialize your task here
		
		exio = new ExternalIOGroup(kUKA_Sunrise_Cabinet_1);	
		
		//nut = new NutRunner(kUKA_Sunrise_Cabinet_1);
		
		int port_atlas = getApplicationData().getProcessData("Port_Atlas").getValue();
		String ip_atlas = getApplicationData().getProcessData("IP_Atlas").getValue();
		
		schunk = new ATC_Schunk(kUKA_Sunrise_Cabinet_1);
		mNutCommThread = new NutCommThread(ip_atlas, port_atlas);
		//                                                           switchESM(ESM_NormalMode);
		mNutCommThread.UsingExio(exio);
		mNut_main = new Thread(mNutCommThread);
		mNut_main.start();
//		tmrEsmCheck = new Timer();	
//		
//		tmrEsmCheck.schedule(tmrTask, 100, 100);
	}

	@Override
	public void run() {
		// your task execution starts here
		int timeSec = 1000;
		int timeMin = 60 * timeSec;
		int timeHour = 60 * timeMin;
		int timeDay = timeHour * 24;
		int timeMonth = timeDay * 30;
		int timeYear = timeMonth * 12;
		
	
		IUserKeyBar keyBar_Schunk = getApplicationUI().createUserKeyBar("Lubber I/O");
		IUserKeyListener keyListner_Schunk_IO = Create_Schunk_Listener();
		
		IUserKeyBar keyBar_Nut = getApplicationUI().createUserKeyBar("Nut I/O");
		IUserKeyListener keyListner_Nut_IO = Create_Nut_Listener();
		
		IUserKeyBar keyBar_Nut_SelectIO1 = getApplicationUI().createUserKeyBar("Nut Sel IO1");
		IUserKeyListener keyListner_Nut_SelectIO1 = Create_Nut_SelectIO_Listener1();
		
		
		// Schunk Keybar
		IUserKey key_Schunk1 = keyBar_Schunk.addUserKey(0, keyListner_Schunk_IO, true);
		key_Schunk1.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_Schunk1.setText(UserKeyAlignment.BottomMiddle, "Open");
		IUserKey key_Schunk2 = keyBar_Schunk.addUserKey(1, keyListner_Schunk_IO, true);
		key_Schunk2.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_Schunk2.setText(UserKeyAlignment.BottomMiddle, "Close");
		IUserKey key_Schunk3 = keyBar_Schunk.addUserKey(2, keyListner_Schunk_IO, true);
		key_Schunk3.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_Schunk3.setText(UserKeyAlignment.BottomMiddle, "InitTool");
		keyBar_Schunk.publish();
		
		// Nut Keybar
		IUserKey key_nut1 = keyBar_Nut.addUserKey(0, keyListner_Nut_IO, true);
		key_nut1.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut1.setText(UserKeyAlignment.BottomMiddle, "Run");
		IUserKey key_nut2 = keyBar_Nut.addUserKey(1, keyListner_Nut_IO, true);
		key_nut2.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut2.setText(UserKeyAlignment.BottomMiddle, "Stop");
		IUserKey key_nut3 = keyBar_Nut.addUserKey(2, keyListner_Nut_IO, true);
		key_nut3.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut3.setText(UserKeyAlignment.BottomMiddle, "Home");
		keyBar_Nut.publish();
		
		// Nut SelectIO Group1 Keybar
		IUserKey key_nut_SelectIO1 = keyBar_Nut_SelectIO1.addUserKey(0, keyListner_Nut_SelectIO1, true);
		key_nut_SelectIO1.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut_SelectIO1.setText(UserKeyAlignment.BottomMiddle, "Nut1");
		IUserKey key_nut_SelectIO2 = keyBar_Nut_SelectIO1.addUserKey(1, keyListner_Nut_SelectIO1, true);
		key_nut_SelectIO2.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut_SelectIO2.setText(UserKeyAlignment.BottomMiddle, "Nut2");
		IUserKey key_nut_SelectIO3 = keyBar_Nut_SelectIO1.addUserKey(2, keyListner_Nut_SelectIO1, true);
		key_nut_SelectIO3.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut_SelectIO3.setText(UserKeyAlignment.BottomMiddle, "Nut3");
		IUserKey key_nut_SelectIO4 = keyBar_Nut_SelectIO1.addUserKey(3, keyListner_Nut_SelectIO1, true);
		key_nut_SelectIO4.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		key_nut_SelectIO4.setText(UserKeyAlignment.BottomMiddle, "Nut4");
		keyBar_Nut_SelectIO1.publish();
		

		// time 1 Year
		ThreadUtil.milliSleep(timeYear);
	}
	
	TimerTask tmrTask = new TimerTask() {
		@Override
		public void run() {
//			if(exio.getIN05())
//			{
//				nut.StopNutRunner();
//				switchESM(ESM_EmergencyStop);
//			}
//			else
//			{
//				switchESM(ESM_NormalMode);
//			}
		}
	};
		
	public IUserKeyListener Create_Schunk_Listener()
	{
		IUserKeyListener Schunk_Listener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {

				if (event == UserKeyEvent.KeyDown) {
					switch (key.getSlot()) {
					case 0:
						//schunk.GripperOpen();
						exio.setOUT06(true);
						ThreadUtil.milliSleep(300);
						exio.setOUT05(false);
						ThreadUtil.milliSleep(300);
						exio.setOUT08(true);
						ThreadUtil.milliSleep(300);
						exio.setOUT07(false);
						
						
						//if(schunk.CheckGripperOpen())
						//{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						//}
						//else
						//{
						//	key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						//}
						
						break;
											 
					case 1:
						//schunk.GripperClose();
						exio.setOUT07(true);
						ThreadUtil.milliSleep(300);
						exio.setOUT08(false);
						ThreadUtil.milliSleep(300);
						exio.setOUT05(true);
						ThreadUtil.milliSleep(300);
						exio.setOUT06(false);

						//ThreadUtil.milliSleep(1000);
						//if(schunk.CheckGripperClose())
						//{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						//}
						//else
						//{
						//	key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						//}
						
						break;
						
					case 2:
						schunk.GripperOpen();
						ThreadUtil.milliSleep(300);
						
						
						//nut.moveNutRunnerHomePos();
						
//						ThreadUtil.milliSleep(1000);
						
						if(exio.getIN14())
						{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}
						else
						{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						
						break;
					default:
						break;
					}
				}
				
			}
		};

		
		return Schunk_Listener;
	}
	
	public IUserKeyListener Create_Nut_Listener()
	{
		IUserKeyListener Nut_Listener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {

				if (event == UserKeyEvent.KeyDown) {
					switch (key.getSlot()) {
					case 0:
						//nut.RunNutRunnerCW();
						mNutCommThread.onSendNutStart();
						key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						break;
						
					case 1:
						//nut.StopNutRunner();
						mNutCommThread.onSendNutStop();
						key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						break;
						
					case 2:
						key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						//nut.moveNutRunnerHomePos();
						mNutCommThread.HomePos();
						ThreadUtil.milliSleep(1000);
						
						if(exio.getIN08())
						{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}
						else
						{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						
						break;
						
					default:
						break;
					}
				}
				
				if (event == UserKeyEvent.KeyUp) {
					switch (key.getSlot()) {
					case 0:
						//nut.StopNutRunner();
						mNutCommThread.onSendNutStop();
						ThreadUtil.milliSleep(300);
						
						if(!exio.getOUT08())
						{
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						
						break;
						
					default:
						break;
					}
				}
			}
		};

		
		return Nut_Listener;
	}
	
	public IUserKeyListener Create_Nut_SelectIO_Listener1()
	{
		IUserKeyListener Nut_SelectIO_Listener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {

				if (event == UserKeyEvent.KeyDown) {
					switch (key.getSlot()) {
					case 0:
						/*if(exio.getOUT11())
						{
							exio.setOUT11(false);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						else
						{
							//nut.selectIO(1);
							mNutCommThread.onSendNutProgramSel(1);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}*/
						mNutCommThread.onSendNutProgramSel(1);
						break;
						
					case 1:
						/*
						if(exio.getOUT12())
						{
							exio.setOUT12(false);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						else
						{
							nut.selectIO(2);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}*/
						mNutCommThread.onSendNutProgramSel(2);
						break;
						
					case 2:
						/*
						if(exio.getOUT11() && exio.getOUT12())
						{
							exio.setOUT11(false);
							exio.setOUT12(false);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						else
						{
							nut.selectIO(3);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}*/
						mNutCommThread.onSendNutProgramSel(3);
						
						break;
						
					case 3:
						/*
						if(exio.getOUT13())
						{
							exio.setOUT13(false);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
						}
						else
						{
							nut.selectIO(4);
							ThreadUtil.milliSleep(300);
							key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
						}*/
						mNutCommThread.onSendNutProgramSel(4);
						break;
						
					default:
						break;
					}
				}
			}
		};
		return Nut_SelectIO_Listener;
	}
	
	private void switchESM(int val) {
		lbr.setESMState( String.valueOf(val) );
	}
}