package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import ioTool.ATC_Schunk;
import ioTool.NutRunner;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.MotionPathCondition;
import com.kuka.roboticsAPI.conditionModel.ReferenceType;
import com.kuka.roboticsAPI.conditionModel.TorqueComponentCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.predefinedCompounds.RobotObserver;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import additionFunction.CallbackReceive;
import additionFunction.ForceTorqueDataSender;
import additionFunction.ForceTorqueDataSender.Type;
import additionFunction.Global;
import additionFunction.NutCommThread;
import additionFunction.TCPClient;

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
public class Manual_Robot extends RoboticsAPIApplication implements CallbackReceive {
	@Inject
	private LBR lbr;

	@Inject
	private Controller kUKA_Sunrise_Cabinet_1;

	@Inject
	@Named("ToolTemplate")
	private Tool tool;

	private ObjectFrame tcp;

	//public static NutRunner nut;
	//public ATC_Schunk schunk;

	NutCommThread mNutCommThread;
	private Thread mNut_main;
	private static int TIGHTENING_STATE = 0;  //0=NONE,1=NOK,2=OK
	private static int DIN_STATE = 0;  //0=NONE,1=NOK,2=OK
	private static int PSET_STATE = 255;  //0~N= ProgramNo 255=ISNONE
	private int port_atlas;
	private String ip_atlas;
	
	
	boolean bInsertResult = false;
	// boolean bNutHomeCheck = false;
	// Nut Runner Job
	private final int NUT_INSERT_15N = 1;
	private final int NUT_LOOSE = 2;
	// private final int FAST_TIGHT = 3;
	// private final int NUT_TIGHT_10N = 4;
	private final int CCW_RUN = 5;
	private final int CW_RUN = 3;
	private final int NUT_TIGHT_50N = 4;
	private final int NUT_ROT_100 = 6;

	static final int Gripper_Rot_Up = 1;
	static final int Gripper_Rot_Down = 2;
	static final int Gripper_Rot_Center = 3;

	private final int IO_SELECT_DELAY = 300;
	private final int IO_SELECT_DELAY_SUB = 100;
	private final int GRIPPER_DELAY = 500;
	private final int NUT_STOP_DELAY = 500;
	private final int TOE_LOOSE_MOVE_DELAY = 100;
	private final int INSERT_NUT_SEEK_DELAY = 100;
	
	// Left or Right Mode Use
	Frame postP;
	public Frame Fr_Rot_Up, Fr_Rot_Down;
	public Frame Fr_Rot_Ready;

	// All Frame Mode Use
	public Frame Fr_Left_Rot_Up, Fr_Left_Rot_Down;
	public Frame Fr_Left_Rot_Ready;
	public Frame Fr_Right_Rot_Up, Fr_Right_Rot_Down;
	public Frame Fr_Right_Rot_Ready;

	Transformation transformation;
	Transformation transformL;
	Transformation transformR;

	private boolean useFTight = false;
	// main Program
	static TCPClient client_main;
	static byte[] SndMsg_main;
	private static boolean thread_main;
	static boolean commOK_main;
	MainPRunnable r_main;
	private Thread t_main;
	private int threadStart_main;
	private int port_main;
	private String ip_main;
	private boolean use_main;
	static Global global;
	private boolean use_adjust;
	private boolean use_vision;
	private boolean use_tight50N;
	private int adjustnochangedelay;
	private double rotatecntoffset;
	private final int THREAD_START = 1;
	private final int THREAD_STOP = 0;
	private boolean use_graph;

	private ExternalIOGroup exio;

	public static final int RECV_MSG_SIZE = 88;
	public static final int SEND_MSG_SIZE = 71;

	public static double LToe_Val, RToe_Val;
	public static double LCamber_Val, RCamber_Val;

	private double PThreshold, NThreshold;

	private double PTestThreshold, NTestThreshold;

	private double ThresholdOffset;

	private double ThresholdLOffset;
	private double ThresholdROffset;

	public static boolean IsValueChange_LH_Toe = true;
	public static boolean IsValueChange_RH_Toe = true;
	public static double LhToeChangeVal = 0.0;
	public static double RhToeChangeVal = 0.0;

	// NCA Value Change Threshold
	private static final double LTOE_CHANGE_THRESHOLD = 0.005;
	private static final double RTOE_CHANGE_THRESHOLD = 0.005;

	public final int ADJ_LEFT = 1;
	public final int ADJ_RIGHT = 2;
	int MANUAL_ONLY_ADJ_PART = 0;

	// Adjust Part
	private final int MODE_NORMAL = 0;
	private final int ADJ_PART_LEFT = 1;
	private final int ADJ_PART_RIGHT = 2;

	public final int Mode_Dissable = 0;
	public final int Mode_Left = 1;
	public final int Mode_Right = 2;
	public final int Mode_All = 5;

	public int NoChangeCount = 0;
	public int Rot_100_Setting_Count = 0;
	public static double ToeChangeVal = 0.0;
	private int totalAdjustCount = 0;

	public final byte SIGNAL_RESET = (byte) 0;
	public final byte SIGNAL_MOVE_VISION_POS = (byte) 1;
	public final byte SIGNAL_MEASURE_VISION_DATA = (byte) 2;
	public final byte SIGNAL_LOOSE_NUT = (byte) 3;
	public final byte SIGNAL_CAMBER_TEST = (byte) 4;
	public final byte SIGNAL_TOE_TEST = (byte) 5;
	public final byte SIGNAL_READY_POS = (byte) 6;
	public final byte SIGNAL_TEST_HOME = (byte) 9;
	public final byte SIGNAL_CAMBER_TOOL = (byte) 41;
	public final byte SIGNAL_CAMBER_ADJ = (byte) 42;
	public final byte SIGNAL_CAMBER_OUT = (byte) 43;
	public final byte SIGNAL_CAMBER_WAITTOOL = (byte) 44;
	public final byte SIGNAL_CAMBER_WAITADJ = (byte) 45;
	public final byte SIGNAL_TOE_TOOL = (byte) 51;
	public final byte SIGNAL_TOE_ADJ = (byte) 52;
	public final byte SIGNAL_TOE_OUT = (byte) 53;
	public final byte SIGNAL_TOE_WAITTOOL = (byte) 54;
	public final byte SIGNAL_TOE_WAITADJ = (byte) 55;
	public static final byte SIGNAL_EXIT = (byte) 100;

	public final byte SIGNAL_TOOL_FAIL = (byte) 21;
	public final byte SIGNAL_ADJ_FAIL = (byte) 61;

	public final byte STATE_RESET = (byte) 0;
	public final byte STATE_VISION_POS = (byte) 1;
	public final byte STATE_MEASURE_VISION_DATA = (byte) 2;
	public final byte STATE_LOOSE_COMPLETE = (byte) 3;
	public final byte STATE_CAMBER_COMPLETE = (byte) 4;
	public final byte STATE_TOE_COMPLETE = (byte) 5;
	public final byte STATE_READY_POS = (byte) 6;
	public final byte STATE_TEST_HOME = (byte) 9;
	// public final byte STATE_TEST_HOME = (byte) 12;

	public final byte STATE_CAMBER_TOOL = (byte) 41;
	public final byte STATE_CAMBER_ADJ = (byte) 42;
	public final byte STATE_CAMBER_WAITTOOL = (byte) 44;
	public final byte STATE_CAMBER_WAITADJ = (byte) 45;
	public final byte STATE_TOE_TOOL = (byte) 51;
	public final byte STATE_TOE_ADJ = (byte) 52;
	public final byte STATE_TOE_WAITTOOL = (byte) 54;
	public final byte STATE_TOE_WAITADJ = (byte) 55;

	public final byte STATE_TOOL_FAIL = (byte) 21;
	public final byte STATE_ADJ_FAIL = (byte) 61;

	public static boolean returnFlag = false;
	private boolean adjustFlag = false;
	private boolean noChangeFlag = false;
	private boolean lastAdjustFlag = false;

	public static int whileMain = 0;

	public Frame changeFrame;

	public double waitChangeMin = 0;
	public double waitChangeMax = 0;

	ForceTorqueDataSender ftSender;

	private boolean toolCamberRetry = false;
	private boolean adjCamberRetry = false;
	private boolean toolToeRetry = false;
	private boolean adjToeRetry = false;

	public static boolean testAbort = false;
	public boolean specIn = false;
	private List<Frame> homeFrame;
	private int statusRobot = 0;
	private static boolean bTriggerCheckFail = false;

	@Override
	public void initialize() {
		// initialize your application here
		// switchESM(1);

		tcp = tool.getFrame("/tcp");
		tool.attachTo(lbr.getFlange());

		exio = new ExternalIOGroup(kUKA_Sunrise_Cabinet_1);

		//schunk = new ATC_Schunk(kUKA_Sunrise_Cabinet_1);

		//nut = new NutRunner(kUKA_Sunrise_Cabinet_1);

		global = new Global();

		initValue();

		initProcessData();

		if (use_main) {
			client_main = new TCPClient(ip_main, port_main);
			ConnectServer();
		}
		mNutCommThread = new NutCommThread(ip_atlas, port_atlas);
		mNutCommThread.UsingCallBack(this);
		mNutCommThread.UsingExio(exio);
		mNut_main = new Thread(mNutCommThread);
		mNut_main.start();

	}

	public void initValue() {
		// Thread Run Init
		thread_main = false;
		commOK_main = false;
		threadStart_main = THREAD_START;

		RToe_Val = 0.0;
		LToe_Val = 0.0;

		RCamber_Val = 0.0;
		LCamber_Val = 0.0;

		ThresholdOffset = 0.0;

		PThreshold = 0.05;
		NThreshold = -0.05;

		ThresholdLOffset = 0;
		ThresholdROffset = 0;

	}

	public void initProcessData() {
		use_main = getApplicationData().getProcessData("Use_Main").getValue();
		port_main = getApplicationData().getProcessData("Port_Main").getValue();
		ip_main = getApplicationData().getProcessData("IP_Main").getValue();

		port_atlas = getApplicationData().getProcessData("Port_Atlas").getValue();
		ip_atlas = getApplicationData().getProcessData("IP_Atlas").getValue();
		
		use_adjust = getApplicationData().getProcessData("AdjustEnable").getValue();
		use_vision = getApplicationData().getProcessData("VisionEnable").getValue();
		use_tight50N = getApplicationData().getProcessData("Tight50NEnable").getValue();
		adjustnochangedelay = getApplicationData().getProcessData("AdjustNoChangeDelay").getValue();
		use_graph = getApplicationData().getProcessData("use_graph").getValue();
		rotatecntoffset = getApplicationData().getProcessData("RotateCntOffset").getValue();

		statusRobot = getApplicationData().getProcessData("statusRobot").getValue();
		
		useFTight = getApplicationData().getProcessData("useFTight").getValue();
		//adjustToolType = getApplicationData().getProcessData("AdjustToolType").getValue();
		//MinSpecOffset = getApplicationData().getProcessData("MinSpecOffset").getValue();
		//MaxSpecOffset = getApplicationData().getProcessData("MaxSpecOffset").getValue();
	}

	public void initTool() {

		getLogger().info("Init Tool");

		
		mNutCommThread.HomePos();

		//schunk.GripperOpen();
	}

	public void initFrame(String direction) {
		ObjectFrame tmp;

		String sFrame = "/P" + global.carType;

		logWrite("<==========" + "CarType : " + global.carType + "==========>", false);

		if (direction.indexOf("Left") >= 0) {
			logWrite("<---- Load Left Frame ---->", false);

			sFrame += "/P1";
			tmp = getApplicationData().getFrame(sFrame + "/P1");
			global.Home = tmp.copyWithRedundancy();

			// Toe
			tmp = getApplicationData().getFrame(sFrame + "/P2/P1");
			global.LeftToe_P1 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P2");
			global.LeftToe_P2 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P3");
			global.LeftToe_P3 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P4");
			global.LeftToe_P4 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P5");
			global.LeftToe_P5 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P6");
			global.LeftToe_P6 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P7");
			global.LeftToe_P7 = tmp.copyWithRedundancy();

			Fr_Rot_Ready = global.LeftToe_P5;
			Fr_Rot_Up = global.LeftToe_P6;
			Fr_Rot_Down = global.LeftToe_P7;

		} else if (direction.indexOf("Right") >= 0) {
			logWrite("<---- Load Right Frame ---->", false);
			sFrame += "/P1";
			tmp = getApplicationData().getFrame(sFrame + "/P1");
			global.Home = tmp.copyWithRedundancy();

			// Toe
			tmp = getApplicationData().getFrame(sFrame + "/P2/P1");
			global.RightToe_P1 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P2");
			global.RightToe_P2 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P3");
			global.RightToe_P3 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P4");
			global.RightToe_P4 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P5");
			global.RightToe_P5 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P6");
			global.RightToe_P6 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P7");
			global.RightToe_P7 = tmp.copyWithRedundancy();

			Fr_Rot_Ready = global.RightToe_P5;
			Fr_Rot_Up = global.RightToe_P6;
			Fr_Rot_Down = global.RightToe_P7;

		} else {
			logWrite("<---- Load All Frame ---->", false);
			sFrame += "/P2";
			tmp = getApplicationData().getFrame(sFrame + "/P1");
			global.Home = tmp.copyWithRedundancy();

			// Toe
			tmp = getApplicationData().getFrame(sFrame + "/P2/P1");
			global.LeftToe_P1 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P2");
			global.LeftToe_P2 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P3");
			global.LeftToe_P3 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P4");
			global.LeftToe_P4 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P5");
			global.LeftToe_P5 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P6");
			global.LeftToe_P6 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P2/P7");
			global.LeftToe_P7 = tmp.copyWithRedundancy();

			Fr_Left_Rot_Ready = global.LeftToe_P5;
			Fr_Left_Rot_Up = global.LeftToe_P6;
			Fr_Left_Rot_Down = global.LeftToe_P7;

			tmp = getApplicationData().getFrame(sFrame + "/P3/P1");
			global.RightToe_P1 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P2");
			global.RightToe_P2 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P3");
			global.RightToe_P3 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P4");
			global.RightToe_P4 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P5");
			global.RightToe_P5 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P6");
			global.RightToe_P6 = tmp.copyWithRedundancy();
			tmp = getApplicationData().getFrame(sFrame + "/P3/P7");
			global.RightToe_P7 = tmp.copyWithRedundancy();

			Fr_Right_Rot_Ready = global.RightToe_P5;
			Fr_Right_Rot_Up = global.RightToe_P6;
			Fr_Right_Rot_Down = global.RightToe_P7;

		}
	}

	private void camberToolIn() {
		global.workState = SIGNAL_CAMBER_TOOL;
	}

	private void camberAdjust() {
		global.workState = SIGNAL_CAMBER_ADJ;
	}

	private boolean toeToolIn() {
		boolean RESULT;
		if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
			if (!toolToeRetry) {
				if (RESULT = toeIn(ADJ_PART_RIGHT)) {
				} else {
					global.workState = STATE_TOOL_FAIL;
				}
				toolToeRetry = true;
			} else {
				if (RESULT = toeRetryIn(ADJ_PART_RIGHT)) {
				} else {
					global.workState = STATE_TOOL_FAIL;
				}
			}
		} else {
			if (!toolToeRetry) {
				if (RESULT = toeIn(ADJ_PART_LEFT)) {
				} else {
					global.workState = STATE_TOOL_FAIL;
				}
				toolToeRetry = true;
			} else {
				if (RESULT = toeRetryIn(ADJ_PART_LEFT)) {
				} else {
					global.workState = STATE_TOOL_FAIL;
				}
			}
		}
		return RESULT;
	}

	private List<Frame> loadFrame(String sPart) {
		List<Frame> tmp = new ArrayList<Frame>();
		if (sPart == "LToe") {
			tmp.add(global.LeftToe_P4);
			tmp.add(global.LeftToe_P3);
			tmp.add(global.LeftToe_P2);
			tmp.add(global.LeftToe_P1);
			tmp.add(global.Home);
		} else if (sPart == "RToe") {
			tmp.add(global.RightToe_P4);
			tmp.add(global.RightToe_P3);
			tmp.add(global.RightToe_P2);
			tmp.add(global.RightToe_P1);
			tmp.add(global.Home);
		}
		return tmp;
	}

	private boolean goReadyposition(String RobotPosition) {
		CartesianImpedanceControlMode ImpedanceMode1 = new CartesianImpedanceControlMode();
		ImpedanceMode1.parametrize(CartDOF.Z).setStiffness(4000);
		ImpedanceMode1.parametrize(CartDOF.X).setStiffness(4000);
		ImpedanceMode1.parametrize(CartDOF.Y).setStiffness(4000);
		ImpedanceMode1.parametrize(CartDOF.ROT).setStiffness(300);

		Frame curP = lbr.getCurrentCartesianPosition(tcp);
		double key = curP.distanceTo(global.Home);

		if (key > 10) {
			homeFrame = new ArrayList<Frame>();
			double curD = lbr.getCurrentCartesianPosition(tcp).getAlphaRad();

			if (RobotPosition == "Right") {
				homeFrame = loadFrame("RToe");
				int shortestFlag = 99;
				double shortestDist = 9999;
				for (int i = 0; i < homeFrame.size(); i++) {
					double dist = curP.distanceTo(homeFrame.get(i));
					if (dist <= shortestDist) {
						shortestFlag = i;
						shortestDist = dist;
					}
				}

				for (int i = shortestFlag; i < homeFrame.size(); i++) {
					tcp.moveAsync(lin(homeFrame.get(i)).setMode(ImpedanceMode1).setJointVelocityRel(0.3));
				}
			} else {
				homeFrame = loadFrame("LToe");
				int shortestFlag = 99;
				double shortestDist = 9999;
				for (int i = 0; i < homeFrame.size(); i++) {
					double dist = curP.distanceTo(homeFrame.get(i));
					if (dist <= shortestDist) {
						shortestFlag = i;
						shortestDist = dist;
					}
				}

				for (int i = shortestFlag; i < homeFrame.size(); i++) {
					tcp.moveAsync(lin(homeFrame.get(i)).setMode(ImpedanceMode1).setJointVelocityRel(0.3));
				}
			}
		}

		tcp.move(lin(global.Home).setJointVelocityRel(0.3));
		return true;
	}
	
	

	private boolean tightNutRunner() {
		boolean tight = false;
		int tightSelect;
		double prev_Val = 0.0;
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		//if (!testAbort) {
			//if (useFTight) {
			//	ToeFastTight();
			//}

			//if (use_tight50N)
				tightSelect = NUT_TIGHT_50N;
			//else
			//	tightSelect = NUT_TIGHT_10N;


			//fastTightAngle();

			if (ToeTight(tightSelect)) {
				logWrite("Tool Tight Type : " + tightSelect + " OK", false);
				tight = true;
			} else {
				logWrite("Tool Tight Type : " + tightSelect + " NOK", true);
				tight = false;
			}
		//}
		return tight;
	}

	private void modificationOnNutTightening() {
		// gripper
		logWrite("-----GRIPPER CLOSE!!", true);
		//schunk.GripperClose();

		CartesianImpedanceControlMode tempCICM = new CartesianImpedanceControlMode();
		tempCICM.parametrize(CartDOF.TRANSL).setStiffness(1500);
		tempCICM.parametrize(CartDOF.ROT).setStiffness(100);
		tempCICM.parametrize(CartDOF.A).setStiffness(300);

		// create target position (+A 30)
		Frame curPosition = lbr.getCurrentCartesianPosition(tcp);
		Frame newTarget = curPosition.copyWithRedundancy().transform(transformation.ofDeg(0, 0, 0, 15, 0, 0));

		// create force condition (torque A)
		double curTorq = lbr.getExternalForceTorque(tcp).getTorque().getZ();
		logWrite("-----CURRENT TORQUE Z = " + curTorq + " !!", true);
		TorqueComponentCondition tempTQ = new TorqueComponentCondition(tcp, CoordinateAxis.Z, curTorq - 20, curTorq + 20);

		// IMotionContainer mc =
		// tcp.move(ptp(newTarget).setJointVelocityRel(0.2).setMode(tempCICM).breakWhen(tempTQ));
		IMotionContainer mc = tcp.move(lin(Fr_Rot_Up).setJointVelocityRel(0.2).setMode(tempCICM).breakWhen(tempTQ));

		if (mc.hasFired(tempTQ)) {
			logWrite("-----BREAK TRIGGERED!!", true);
			curTorq = lbr.getExternalForceTorque(tcp).getTorque().getZ();
			logWrite("-----[AFTER] CURRENT TORQUE Z = " + curTorq + " !!", true);
		} else {
			logWrite("-----BREAK NOT TRIGGERED!!", true);
		}

		// gripper
		logWrite("-----GRIPPER OPEN!!", true);
		//schunk.GripperOpen();
	}
	
	/*private boolean ToeFastTight() {
		boolean bResult = false;
		int nCount = 0;
		int nDagi = 0;
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.Z).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8).setMode(ctim));
		ThreadUtil.milliSleep(50);
		try {
			logWrite("Fast Tight", true);
			//nut.selectIO(FAST_TIGHT);
			mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
			int cnt = 0;
			while(PSET_STATE == 6){
				if(cnt == 99){
					mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
				}
				if(cnt > 199){
					cnt = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cnt++;
			}
			ThreadUtil.milliSleep(IO_SELECT_DELAY);

			tcp.move(linRel(0, 0, 10).setMode(ctim).setJointVelocityRel(0.5));

			int start = 1;
			while (start > 0) {
				if (nCount > 3) {
					logWrite("FastRun Count : " + nCount, true);
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					return false;
				}
				//nut.RunNutRunnerCW();
				TIGHTENING_STATE = 0;
				mNutCommThread.onSendNutStart();
				//while (!nut.CheckTightenNOK() && !nut.CheckTightenOK()) {
				while (TIGHTENING_STATE != 2 ) {
					if (nDagi > 300) {
						nDagi = 0;
						return false;
					}
					nDagi++;
					ThreadUtil.milliSleep(10);
				}
				nDagi = 0;
				//nut.StopNutRunner();
				mNutCommThread.onSendNutStop();
				//if (nut.CheckTightenNOK()) {
				if (TIGHTENING_STATE == 1) {
					start = 0;
					break;
				} else
					nCount++;

				//while (nut.CheckTightenNOK() || nut.CheckTightenOK()) {
				while (TIGHTENING_STATE != 0) {
					if (nDagi > 300) {
						nDagi = 0;
						return false;
					}
					nDagi++;
					ThreadUtil.milliSleep(10);
				}
				nDagi = 0;
			}

			//nut.StopNutRunner();
			mNutCommThread.onSendNutStop();
			TIGHTENING_STATE = 0;
			logWrite("Tool Rotation 1300 Complete", true);
			bResult = true;
		} catch (Exception ex) {
			getLogger().error(ex.toString());
		}
		return bResult;
	}*/

	private void toeAdjust() {
		calcThreshold();
		logWrite("Toe Align", false);

		boolean adjustOk = false;
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		if (!specIn) {
			if (use_adjust) {
				SetRotTransformation();

				if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
					if (!toeTest(ADJ_PART_RIGHT)) {
						logWrite("Toe Right Adjust Error", true);
						global.workState = STATE_ADJ_FAIL;
					} else {
						if (!testAbort) {
							adjustOk = true;
							global.workState = SIGNAL_TOE_ADJ;
						}
					}
				} else {
					if (!toeTest(ADJ_PART_LEFT)) {
						logWrite("Toe Left Adjust Error", true);
						global.workState = STATE_ADJ_FAIL;
					} else {
						if (!testAbort) {
							adjustOk = true;
							global.workState = SIGNAL_TOE_ADJ;
						}
					}
				}/*
				 * else if (global.adjustPart == 0) { if (!toeTest()) {
				 * logWrite("Toe Right Adjust Error", true); global.workState =
				 * STATE_ADJ_FAIL; } else { if (!testAbort) { adjustOk = true;
				 * global.workState = SIGNAL_TOE_ADJ; } } }
				 */

				//schunk.GripperOpen();

				logWrite("Rot End, Return ROT Ready Pos", false);
				tcp.move(lin(Fr_Rot_Ready).setMode(ctim).setJointVelocityRel(0.6));
			} else {
				logWrite("use Adjust select False", true);
				global.workState = SIGNAL_TOE_ADJ;
			}
		} else {
			//schunk.GripperOpen();

			logWrite("Rot End, Return ROT Ready Pos", false);
			tcp.move(lin(Fr_Rot_Ready).setMode(ctim).setJointVelocityRel(0.6));

			for (int y = 0; y < 3; y++) {
				if (tightNutRunner()) {
					// break;
				}
			}
			global.workState = SIGNAL_TOE_ADJ;
		}
	}

	private void statusRobotSave(int status) {
		getApplicationData().getProcessData("statusRobot").setValue(status);
	}
	
	private void NutTest(int nutDirection){
		if(nutDirection == 0){ //nut.selectIO(4); //cw
			mNutCommThread.onSendNutProgramSel(CW_RUN);
			int cntss = 0;
			while(PSET_STATE == 3){
				if(cntss == 99){
					mNutCommThread.onSendNutProgramSel(CW_RUN);
				}
				if(cntss > 199){
					cntss = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cntss++;
			}
		}
		else{ //nut.selectIO(3); //ccw
			mNutCommThread.onSendNutProgramSel(CCW_RUN);
			int cntss = 0;
			while(PSET_STATE == 5){
				if(cntss == 99){
					mNutCommThread.onSendNutProgramSel(CCW_RUN);
				}
				if(cntss > 199){
					cntss = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cntss++;
			}
		}
		ThreadUtil.milliSleep(IO_SELECT_DELAY);
		
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(4500);
		ctim.parametrize(CartDOF.Y).setStiffness(4500);
		ctim.parametrize(CartDOF.Z).setStiffness(4500);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.5));
		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1, TimeUnit.MILLISECONDS));
		exio.setOUT05(false);
		ThreadUtil.milliSleep(300);
		exio.setOUT06(true);
		ThreadUtil.milliSleep(300);
		exio.setOUT07(false);
		ThreadUtil.milliSleep(300);
		exio.setOUT08(true);
		ThreadUtil.milliSleep(300);
		//nut.RunNutRunnerCW();
		mNutCommThread.onSendNutStart();

		int nDagi = 0;
		
		while (true) {
			if (nDagi > 1200) {
				nDagi = 0;
				break;
			}
			nDagi++;
			ThreadUtil.milliSleep(10);
		}

		mNutCommThread.onSendNutStop();
		exio.setOUT05(true);
		exio.setOUT06(false);
		ThreadUtil.milliSleep(1000);
		exio.setOUT07(true);
		exio.setOUT08(false);
		ThreadUtil.milliSleep(1000);
		hold.cancel();
		
		
		
	}
	
	

	@Override
	public void run() {

		if (use_graph) {
			ftSender = new ForceTorqueDataSender(lbr, tcp, 30000, -1, 20, Type.TOE, getLogger());
			ftSender.start();
		}

		int whileMain = 1;
		int whileSub = 1;
		int whileSub1 = 1;
		int uiKey = 0;

		// your application execution starts here
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		String RobotPosition = "Left";

		while (whileMain > 0) {
			if (use_main == false) {
				uiKey = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Cartype?", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "Exit");
				if (uiKey == 10)
					whileSub1 = 0;
				else if (uiKey == 0)
					global.carType = 1;
				else if (uiKey == 1)
					global.carType = 2;
				else if (uiKey == 2)
					global.carType = 3;
				else if (uiKey == 3)
					global.carType = 4;
				else if (uiKey == 4)
					global.carType = 5;
				else if (uiKey == 5)
					global.carType = 6;
				else if (uiKey == 6)
					global.carType = 7;
				else if (uiKey == 7)
					global.carType = 8;
				else if (uiKey == 8)
					global.carType = 9;
				else if (uiKey == 9)
					global.carType = 10;
			}
			whileSub = 1;
			// select frame
			uiKey = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Robot Position?", "Left", "Right");
			if (uiKey == 0) {
				initFrame("Left");
				RobotPosition = "Left";
				MANUAL_ONLY_ADJ_PART = 1;
			} else if (uiKey == 1) {
				initFrame("Right");
				RobotPosition = "Right";
				MANUAL_ONLY_ADJ_PART = 2;
			}

			while (whileSub > 0) {
				whileSub1 = 1;
				uiKey = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Select Test?", "Tool Test", "Nut Test", "Exit");
				if (uiKey == 0) {
					while (whileSub1 > 0) {
						uiKey = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Select Test?", "Tool In", "Tool Out", "Tool Loose", "Tool Tight", "reloose",
								"CW", "CCW", "Exit");
						if (RobotPosition == "Left") {
							if (uiKey == 0) {
								tcp.moveAsync(lin(global.LeftToe_P1).setJointVelocityRel(0.6).setBlendingRel(0.3));
								visionTransform();
								toeIn(ADJ_PART_LEFT);
							} else if (uiKey == 1)
								toeOut(ADJ_PART_LEFT);
							else if (uiKey == 2)
								ToeLoose(NUT_LOOSE);
							else if (uiKey == 3)
								tightNutRunner();
							else if (uiKey == 4) {
								//toeReLoose();
							}else if (uiKey == 5) {
								//toeReLoose();4
								NutTest(0);
								
							}else if (uiKey == 6) {
								NutTest(1);
								
								//toeReLoose();
							} else
								whileSub1 = 0;
						} else {
							if (uiKey == 0) {
								tcp.moveAsync(lin(global.RightToe_P1).setJointVelocityRel(0.6).setBlendingRel(0.3));
								visionTransform();
								toeIn(ADJ_PART_RIGHT);
							} else if (uiKey == 1)
								toeOut(ADJ_PART_RIGHT);
							else if (uiKey == 2)
								ToeLoose(NUT_LOOSE);
							else if (uiKey == 3)
								tightNutRunner();
							else if (uiKey == 4) {
								//toeReLoose();
							} else
								whileSub1 = 0;
						}
					}
				} else if (uiKey == 1) {
					while (whileSub1 > 0) {
						uiKey = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "nut Job Change", "1", "2", "3", "4", "5", "6", "7", "Exit", "CW", "Stop");
						if (uiKey == 7)
							whileSub1 = 0;
						else if (uiKey == 8){
							logWrite("[ORIGIN] LTOEVAL: "+ LToe_Val + "  RTOEVAL: "+ RToe_Val);
							//nut.RunNutRunnerCW();
							TIGHTENING_STATE = 0;
							mNutCommThread.onSendNutStart();
							int nDagi = 0;
							double PastLtoe = LToe_Val;
							double PastRtoe = RToe_Val;
							//while (!nut.CheckTightenNOK() && !nut.CheckTightenOK()) {
							while(TIGHTENING_STATE == 0){
								if (nDagi > 1500) {
									nDagi = 0;
									break;
								}
								nDagi++;
								ThreadUtil.milliSleep(10);
							}
							logWrite("[DIFFERENT]LTOEVAL: "+ Math.abs(LToe_Val - PastLtoe) + "  RTOEVAL: "+ Math.abs(RToe_Val - PastRtoe));
						}
						else if (uiKey == 9){
							//nut.StopNutRunner();
							mNutCommThread.onSendNutStop();
							TIGHTENING_STATE = 0;
						}
						
						else{
							//nut.selectIO(uiKey + 1);
							mNutCommThread.onSendNutProgramSel(uiKey+1);
						}
						

					}
				} else {
					whileSub = 0;
					whileMain = 0;
				}
			}

		}

		thread_main = false;
		threadStart_main = THREAD_STOP;

	}

	/*****************************************************************
	 * 
	 * Common Function
	 * 
	 */


	/*public boolean fastTightAngle(){
		int nDagi = 0;
		logWrite("Fast Tight angle", true);
		//nut.selectIO(FAST_TIGHT);
		mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
		int cnt = 0;
		while(PSET_STATE == 6){
			if(cnt == 99){
				mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
			}
			if(cnt > 199){
				cnt = 0;
				break;
			}
			ThreadUtil.milliSleep(10);
			cnt++;
		}
		ThreadUtil.milliSleep(IO_SELECT_DELAY);
		
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.Z).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8).setMode(ctim));
		ThreadUtil.milliSleep(50);

		tcp.move(linRel(0, 0, 10).setMode(ctim).setJointVelocityRel(0.5));
		
		
//		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1, TimeUnit.MILLISECONDS));
		
		//nut.RunNutRunnerCW();
		TIGHTENING_STATE = 0;
		mNutCommThread.onSendNutStart();
		
		//while (!nut.CheckTightenNOK() && !nut.CheckTightenOK()) {
		while(TIGHTENING_STATE == 0){
			if (nDagi > 500) {
				nDagi = 0;
				return false;
			}
			nDagi++;
			ThreadUtil.milliSleep(10);
		}
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		TIGHTENING_STATE = 0;
		
		nDagi = 0;
		
//		nut.StopNutRunner();
		
//		hold.cancel();
		logWrite("Fast Tight angle Complete", true);
		return true;
	}*/
	
	/*public boolean toeReLoose() {
		boolean looseOk = false;

//		GripperHold();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		try {
			
			fastTightAngle();
			ThreadUtil.milliSleep(500);

			if (!ToeTight(NUT_TIGHT_10N)) {
				logWrite("10N Tighte fail", true);
				return false;
			}

			ThreadUtil.milliSleep(500);

			if (!ToeLoose(NUT_LOOSE)) {
				logWrite("loose fail", true);
				return false;
			}
			looseOk = true;
		} catch (Exception ex) {
			getLogger().error(ex.toString());
		}

		return looseOk;
	}*/

	/*private void GripperHold() {
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(3500);
		ctim.parametrize(CartDOF.Y).setStiffness(3500);
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.ROT).setStiffness(200);

		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1, TimeUnit.MILLISECONDS));
		schunk.GripperClose();

		ThreadUtil.milliSleep(GRIPPER_DELAY);
		// Mode Change Position Control
		hold.cancel();
		schunk.GripperOpen();
		// tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.1));
		System.out.println("Gripper Hold Function end");
	}*/

	public void calcThreshold() {

		PThreshold = getApplicationData().getProcessData("PSpec").getValue();
		NThreshold = getApplicationData().getProcessData("NSpec").getValue();
		ThresholdLOffset = getApplicationData().getProcessData("ThresholdLOffset").getValue();
		ThresholdROffset = getApplicationData().getProcessData("ThresholdROffset").getValue();

		if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
			if (RToe_Val > PThreshold) {
				PTestThreshold = PThreshold;
				NTestThreshold = NThreshold;
			} else if (RToe_Val < NThreshold) {
				ThresholdOffset = ThresholdROffset;
				PTestThreshold = PThreshold + ThresholdOffset;
				NTestThreshold = NThreshold + ThresholdOffset;

			}
		} else {
			if (LToe_Val > PThreshold) {
				PTestThreshold = PThreshold;
				NTestThreshold = NThreshold;
			} else if (LToe_Val < NThreshold) {
				ThresholdOffset = ThresholdLOffset;
				PTestThreshold = PThreshold + ThresholdOffset;
				NTestThreshold = NThreshold + ThresholdOffset;
			}
		}
		if (MANUAL_ONLY_ADJ_PART == ADJ_PART_LEFT) {
			logWrite("Spec In No Loose Nut", true);
			logWrite("LToe_Val:" + LToe_Val, true);
			specIn = isBetweenSpec(LToe_Val, NTestThreshold, PTestThreshold);
		} else if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
			logWrite("Spec In No Loose Nut", true);
			logWrite("RToe_Val:" + RToe_Val, true);
			specIn = isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold);
		} else {
			logWrite("Spec In No Loose Nut", true);
			logWrite("RToe_Val:" + RToe_Val, true);
			specIn = isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold);
		}

		logWrite("PThreshold : " + PTestThreshold, true);
		logWrite("NThreshold : " + NTestThreshold, true);

	}

	private boolean checkDataNoChange(double prevData, double curData) {
		double ToeValDif;

		ToeValDif = Math.abs(prevData - curData);
		logWrite("Toe Diff : " + ToeValDif, true);

		return ToeValDif < 0.003;
	}

	private void waitDataNoChange(int nDir, double min, double max, int delay) {
		double val;

		if (nDir == ADJ_LEFT) {
			val = LToe_Val;
		} else {
			val = RToe_Val;
		}

		if (isBetweenSpec(val, min, max)) {
			ThreadUtil.milliSleep(delay);
		}
	}

	// Spec Check
	public static boolean isBetweenSpec(double value, double min, double max) {
		return value >= min && value <= max;
	}

	// Home Sensor Check
	public boolean checkHome() {
		return exio.getIN07();
	}

	private void logWrite(String log, boolean err) {
		if (err)
			getLogger().error(log);
		else
			getLogger().info(log);
	}

	private void logWrite(String log) {
		// if (useLog) {
		getLogger().info(log);
		// }
	}

	/*****************************************************************
	 * 
	 * Move Home Position
	 * 
	 * @return
	 */

	public void MoveHomePosition() {
		tcp.move(lin(global.Home).setJointVelocityRel(0.6));
		getLogger().info("Move Home");

		if (checkHome()) {
			getLogger().info("Home - Complete");

			if (returnFlag == true) {
				logWrite("Return Flag True", false);
			}
		}
	}

	public boolean toeRetryIn(int direction) {
		boolean bInserted = false;
		Frame p4;
		if (direction == ADJ_PART_RIGHT) {
			p4 = global.RightToe_P4;
		} else {
			p4 = global.LeftToe_P4;
		}
		tcp.move(lin(p4).setJointVelocityRel(0.5));
		getLogger().info("Move p4 TieRoad Inner");

		//GripperHold();

		// Detect Nut
		if (DetectTighteningNut()) {
			// Check Inserted
			bInserted = checkNutInserted(direction);
		}
		bInserted = true;
		return bInserted;
	}

	public boolean toeIn(int direction) {
		boolean bInserted = false;
		Frame p2 = null;
		Frame p3 = null;
		Frame p4 = null;
		if (direction == ADJ_PART_RIGHT) {
			p2 = global.RightToe_P2;
			p3 = global.RightToe_P3;
			p4 = global.RightToe_P4;
		} else {
			p2 = global.LeftToe_P2;
			p3 = global.LeftToe_P3;
			p4 = global.LeftToe_P4;
		}

		//nut.selectIO(NUT_INSERT_15N);
		mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
		int cnt = 0;
		while(PSET_STATE == 1){
			if(cnt == 99){
				mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
			}
			if(cnt > 199){
				cnt = 0;
				break;
			}
			ThreadUtil.milliSleep(10);
			cnt++;
		}
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(4000);
		ctim.parametrize(CartDOF.Y).setStiffness(3000);
		ctim.parametrize(CartDOF.Z).setStiffness(4000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		CartesianSineImpedanceControlMode csim = new CartesianSineImpedanceControlMode();
		csim.parametrize(CartDOF.X).setStiffness(2800);
		csim.parametrize(CartDOF.Y).setStiffness(2800).setAmplitude(10).setFrequency(4);
		csim.parametrize(CartDOF.Z).setStiffness(4000);
		csim.parametrize(CartDOF.ROT).setStiffness(300);

		// Move Frame Insert Ready Position
		tcp.moveAsync(lin(p2).setJointVelocityRel(0.6).setBlendingRel(0.3));
		getLogger().info("Move P2 Approach");
		tcp.move(lin(p3).setJointVelocityRel(0.6));
		getLogger().info("Move Approach p3");

		// Move to TieRoad
		double currentFX = lbr.getExternalForceTorque(tcp).getForce().getX();
		ForceComponentCondition fcx = new ForceComponentCondition(tcp, CoordinateAxis.X, currentFX - 20, currentFX + 20);

		IMotionContainer mc = tcp.move(lin(p4).setJointVelocityRel(0.1).setMode(ctim).breakWhen(fcx));

		// TODO : sk.kim 190506_1430
		/*
		 * if (!mc.hasFired(fcx)) { ForceComponentCondition fcx2 = new
		 * ForceComponentCondition(tcp, CoordinateAxis.X, currentFX - 20,
		 * currentFX + 20); IMotionContainer mc2 = tcp.move(linRel(-20.0, 0.0,
		 * 0.0).setJointVelocityRel(0.1).setMode(ctim).breakWhen(fcx2));
		 * 
		 * if (!mc2.hasFired(fcx)) { ForceComponentCondition fcx3 = new
		 * ForceComponentCondition(tcp, CoordinateAxis.X, currentFX - 20,
		 * currentFX + 20); IMotionContainer mc3 = tcp.move(linRel(-20.0, 0.0,
		 * 0.0).setJointVelocityRel(0.1).setMode(ctim).breakWhen(fcx2)); } }
		 */
		if (mc.hasFired(fcx)) {
			logWrite("Move P4(TieRod) BREAKED!!" + fcx, true);
		}
		logWrite("Trying oscilation!!", true);
		ForceComponentCondition fcx2 = new ForceComponentCondition(tcp, CoordinateAxis.X, currentFX - 35, currentFX + 35);
		IMotionContainer mc2 = tcp.move(linRel(-70.0, 0.0, 0.0).setJointVelocityRel(0.1).setMode(ctim).breakWhen(fcx2));
		// }

		if (mc2.hasFired(fcx2)) {
			logWrite("Move X Direction (TieRod) BREAKED!!" + fcx2, true);
		}

		//GripperHold();

		// Detect Nut
		if (DetectTighteningNut()) {
			// Check Inserted
			bInserted = checkNutInserted(direction);
		}

		return bInserted;
	}

	public boolean checkNutInserted(int direction) {
		boolean bInserted = false;
		int nMaxTryCount = 5;

		int nTryCount = 0;
		bInserted = false;

		while (!bInserted) {
			bInserted = InsertNutRunner();

			if (!bInserted) {
				if (nTryCount > nMaxTryCount) {
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					break;
				}

				nTryCount++;
				logWrite("NutInsert Try Count : " + nTryCount, true);
				ThreadUtil.milliSleep(INSERT_NUT_SEEK_DELAY);
			}
		}

		return bInserted;
	}

	public double getDetectedDistance(int direction) {
		double distance = 0.0;

		Frame target = null;
		if (direction == ADJ_PART_RIGHT) {
			target = global.RightToe_P4.copyWithRedundancy();
		} else {
			target = global.LeftToe_P4.copyWithRedundancy();
		}

		distance = Math.abs(lbr.getCurrentCartesianPosition(tcp, target).getZ());
		logWrite("Distance : " + distance, false);

		return distance;
	}

	/*****************************************************************
	 * 
	 * Toe Left Out
	 * 
	 * @return
	 */

	public boolean toeOut(int direction) {
		boolean bResult = false;
		int nTryCount = 0;
		int nMaxTryCount = 500;
		double dbRot = 0;
		if (direction == ADJ_PART_RIGHT) {
			dbRot = 2.0;
		} else {
			dbRot = -2.0; // ToolOut Rot Angle
		}

		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(2500);
		ctim.parametrize(CartDOF.X).setStiffness(2500);
		ctim.parametrize(CartDOF.Y).setStiffness(2500);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		CartesianImpedanceControlMode ctim1 = new CartesianImpedanceControlMode();
		ctim1.parametrize(CartDOF.TRANSL).setStiffness(5000);
		ctim1.parametrize(CartDOF.ROT).setStiffness(300);

		CartesianSineImpedanceControlMode csim = new CartesianSineImpedanceControlMode();
		csim.parametrize(CartDOF.Z).setStiffness(5000);
		csim.parametrize(CartDOF.X).setStiffness(2000);
		csim.parametrize(CartDOF.Y).setStiffness(2000);
		csim.parametrize(CartDOF.ROT).setStiffness(300);
		csim.parametrize(CartDOF.C).setStiffness(300).setAmplitude(10).setFrequency(2);

		//GripperHold();
		Frame rotationP1;
		Frame rotationP2;
		Frame target;

		rotationP1 = lbr.getCurrentCartesianPosition(tcp);
		rotationP2 = rotationP1.copyWithRedundancy().transform(tcp, Transformation.ofDeg(0, 0, 0, dbRot, 0, 0));
		target = rotationP1.copyWithRedundancy().transform(tcp, Transformation.ofTranslation(0, 0, -20));
		tcp.move(lin(rotationP2).setJointVelocityRel(0.5).setMode(ctim));
		logWrite("Move Rot A : " + dbRot, false);
		tcp.move(lin(target).setJointVelocityRel(0.4).setMode(csim));
		getLogger().info("Move P4 Approach");
		tcp.move(lin(target).setJointVelocityRel(0.4).setMode(ctim1));
		getLogger().info("Move P4 Approach");
		tcp.move(lin(target).setJointVelocityRel(0.4));
		getLogger().info("Move P4 Approach");
		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000, 3000, 3000, 3000, 3000, 200, 10);
		IMotionContainer hold = tcp.moveAsync(positionHold(jicm, -1, TimeUnit.MILLISECONDS));

		boolean check = false;
		//schunk.GripperClose();
		for (int i = 0; i < 10; i++) {
			check = mNutCommThread.HomePos();
			if (check) {
				if (exio.getIN14())
					break;
				else
					check = false;
			}
		}
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		//schunk.GripperOpen();
		ThreadUtil.milliSleep(GRIPPER_DELAY);
		hold.cancel();
		if (exio.getIN14()) {
			System.out.println("Nut Runner Home - OK");
			moveOutPosition(direction);
			MoveHomePosition();
			System.err.println("Tool Out Success");
			bResult = true;
		} else {
			// nut.StopNutRunner();
			System.out.println("Nut Runner Home - Fail");
		}
		return bResult;
	}

	public void moveOutPosition(int direction) {
		Frame p2;
		Frame p1;
		if (direction == ADJ_PART_RIGHT) {
			p2 = global.RightToe_P2;
			p1 = global.RightToe_P1;
		} else {
			p2 = global.LeftToe_P2;
			p1 = global.LeftToe_P1;
		}
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(3000);
		ctim.parametrize(CartDOF.Y).setStiffness(3000);
		ctim.parametrize(CartDOF.Z).setStiffness(3000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		CartesianSineImpedanceControlMode csim = new CartesianSineImpedanceControlMode();
		csim.parametrize(CartDOF.X).setStiffness(5000);
		csim.parametrize(CartDOF.Y).setStiffness(5000).setAmplitude(10).setFrequency(4);
		csim.parametrize(CartDOF.Z).setStiffness(5000);
		csim.parametrize(CartDOF.A).setStiffness(100);
		csim.parametrize(CartDOF.B).setStiffness(300);
		csim.parametrize(CartDOF.C).setStiffness(300);

		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000, 3000, 3000, 3000, 3000, 3000, 10);

		double currentFX = lbr.getExternalForceTorque(tcp).getForce().getX();
		ForceComponentCondition fcx = new ForceComponentCondition(tcp, CoordinateAxis.X, currentFX - 30, currentFX + 30);

		IMotionContainer mc = tcp.move(linRel(50, 0, 0).setJointVelocityRel(0.1).setMode(csim).breakWhen(fcx));
		getLogger().info("Move P3 Insert Ready");
		if (mc.hasFired(fcx)) {
			logWrite("Detected Colision 15Nm", true);
			// tcp.move(linRel(0, 1.0, 0.0).setJointVelocityRel(0.1));
			tcp.move(linRel(30, 0, 0).setJointVelocityRel(0.1).setMode(jicm));
		}
		tcp.moveAsync(lin(p2).setJointVelocityRel(0.6).setBlendingCart(5));
		getLogger().info("Move P2 Approach");
		tcp.move(lin(p1).setJointVelocityRel(0.6));
		getLogger().info("Move P1 Approach");

		returnFlag = true;
	}

	/*****************************************************************
	 * 
	 * Right Tool Insert Nut
	 * 
	 * @return
	 */

	public double getRightDetectedDistance() {
		double distance = 0.0;

		Frame target = global.RightToe_P4.copyWithRedundancy();

		distance = Math.abs(lbr.getCurrentCartesianPosition(tcp, target).getZ());
		logWrite("Distance : " + distance, false);

		return distance;
	}

	public boolean checkRightNutInserted() {
		double distance = 0.0;
		boolean bInserted = false;
		int nMaxTryCount = 5;
		double inDistance = 23.0;

		distance = getRightDetectedDistance();

		if (distance > inDistance) {
			logWrite("Tool Inserted One Time", false);
			bInserted = InsertNutRunner(); // YC edit
		} else {
			int nTryCount = 0;
			bInserted = false;

			while (!bInserted) {
				bInserted = InsertNutRunner();

				if (nTryCount > nMaxTryCount) {
					break;
				}

				nTryCount++;
				logWrite("Try Count : " + nTryCount, true);
			}

			ThreadUtil.milliSleep(INSERT_NUT_SEEK_DELAY);
		}

		return bInserted;
	}

	/*****************************************************************
	 * 
	 * Detect Nut
	 * 
	 * @return
	 */

	private boolean DetectTighteningNut() {
		try {
			IMotionContainer mc = null;

			CartesianImpedanceControlMode csim = new CartesianImpedanceControlMode();
			csim.parametrize(CartDOF.Z).setStiffness(3000);
			csim.parametrize(CartDOF.X).setStiffness(3000);
			csim.parametrize(CartDOF.Y).setStiffness(3000);
			csim.parametrize(CartDOF.ROT).setStiffness(300);

			// get current Robot Force Z
			double currentForceZ = lbr.getExternalForceTorque(tcp).getForce().getZ();

			ForceComponentCondition fcZ = new ForceComponentCondition(tcp, CoordinateAxis.Z, currentForceZ - 35.0, currentForceZ + 35.0);

			mc = tcp.move(linRel(0.0, 0.0, 50.0).setJointVelocityRel(0.4).breakWhen(fcZ).setMode(csim));

			if (mc.hasFired(fcZ)) {
				logWrite("Detected Tightening Nut", true);

				return true;
			}

		} catch (Exception ex) {

		}

		return false;
	}

	/*****************************************************************
	 * 
	 * Insert Nut Runner to Tight Nut (LF, RF Same)
	 * 
	 * @return
	 */

	private boolean InsertNutRunner() {
		int cnt = 0;
		bTriggerCheckFail = false;
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		CartesianSineImpedanceControlMode sineMode = new CartesianSineImpedanceControlMode();
		sineMode.parametrize(CartDOF.X).setStiffness(2800).setAmplitude(10).setFrequency(2).setDamping(1.0);
		sineMode.parametrize(CartDOF.Y).setStiffness(2800).setAmplitude(10).setFrequency(2).setDamping(1.0);
		sineMode.parametrize(CartDOF.Z).setStiffness(2000).setBias(20).setDamping(1.0);
		sineMode.parametrize(CartDOF.A).setStiffness(300);
		sineMode.parametrize(CartDOF.B).setStiffness(300);
		sineMode.parametrize(CartDOF.C).setStiffness(300);

		CartesianSineImpedanceControlMode sineMode2 = new CartesianSineImpedanceControlMode();
		sineMode2.parametrize(CartDOF.X).setStiffness(3000);
		sineMode2.parametrize(CartDOF.Y).setStiffness(3000);
		sineMode2.parametrize(CartDOF.Z).setStiffness(2000).setBias(20);
		// TODO : sk.kim 190506_1327
		sineMode2.parametrize(CartDOF.A).setStiffness(/* 300 */100).setAmplitude(/* 5 */3).setFrequency(1);
		sineMode2.parametrize(CartDOF.B).setStiffness(300);
		sineMode2.parametrize(CartDOF.C).setStiffness(300);

		try {
			bInsertResult = false;

			logWrite("Start Insert Nut Runner");

			ICallbackAction action = new ICallbackAction() {
				@Override
				public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
					while (!triggerInformation.getMotionContainer().isFinished()) {
						switch(TIGHTENING_STATE){
						case 1:
							bInsertResult = false;
							//mNutCommThread.onSendNutStop();
							TIGHTENING_STATE = 0;
							logWrite("Nut Insert - NOK", true);
							triggerInformation.getMotionContainer().cancel();
							break;
						case 2:
							bInsertResult = true;
							bTriggerCheckFail = true;
							mNutCommThread.onSendNutStop();
							TIGHTENING_STATE = 0;
							logWrite("Nut Insert - OK", false);
							triggerInformation.getMotionContainer().cancel();
							break;
							
					}
						/*if (TIGHTENING_OK) {
							bInsertResult = true;
							bTriggerCheckFail = true;
							//nut.StopNutRunner();
							mNutCommThread.onSendNutStop();
							logWrite("Nut Insert - OK", false);
							TIGHTENING_OK = false;
							triggerInformation.getMotionContainer().cancel();
							break;
						} else if (!TIGHTENING_OK) {
							bInsertResult = false;
							//nut.StopNutRunner();
							mNutCommThread.onSendNutStop();
							logWrite("Nut Insert - NOK", true);
							TIGHTENING_OK = false;
							triggerInformation.getMotionContainer().cancel();
							break;
						}*/
					}
				}
			};

			//nut.RunNutRunnerCW();
			TIGHTENING_STATE = 0;
			mNutCommThread.onSendNutStart();

			tcp.move(linRel(0.0, 0.0, 20.0).setMode(sineMode).setJointVelocityRel(0.1).triggerWhen(new MotionPathCondition(ReferenceType.START, 0, 0), action));
		} catch (Exception ex) {
			logWrite(ex.toString(), true);
		}

		while (!bTriggerCheckFail) {
			cnt++;
			if (cnt > 500) {
				logWrite("Tight Signal Read Fail : InsertNutRunner", true);
				return false;
			}
			switch(TIGHTENING_STATE){
			case 1:
				bInsertResult = false;
				//mNutCommThread.onSendNutStop();
				//TIGHTENING_STATE = 0;
				logWrite("Nut Trigger After Insert - NOK", true);
				bTriggerCheckFail = false;
				break;
			case 2:
				bInsertResult = true;
				bTriggerCheckFail = true;
				mNutCommThread.onSendNutStop();
				logWrite("Nut Trigger After Insert - OK", false);
				break;
				
			}
			/*
			if (TIGHTENING_OK) {
				bInsertResult = true;
				//nut.StopNutRunner();
				mNutCommThread.onSendNutStop();
				logWrite("Nut Insert - OK", false);
				TIGHTENING_OK = false;
				bTriggerCheckFail = true;
			} else if (!TIGHTENING_OK) {
				bInsertResult = false;
				//nut.StopNutRunner();
				mNutCommThread.onSendNutStop();
				bTriggerCheckFail = true;
				TIGHTENING_OK = false;
				logWrite("Nut Insert - NOK", true);
			}*/
			ThreadUtil.milliSleep(10);
		}

		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		TIGHTENING_STATE = 0;
		tcp.move(linRel(0.0, 0.0, 20.0).setMode(sineMode2).setCartVelocity(40));
		return bInsertResult;
	}

	private boolean toeTest(int direction) {
		double Toe_Val = 0;
		int repeatCount;
		boolean IsAdjusted;
		int Adjust_Dir = 0;

		if (lastAdjustFlag) {
			PTestThreshold = PThreshold;
			NTestThreshold = NThreshold;
		}
		logWrite("Toe Value : " + LToe_Val, false);

		repeatCount = 0;
		IsAdjusted = false;
		if (direction == ADJ_PART_RIGHT) {
			Toe_Val = RToe_Val;
		} else
			Toe_Val = LToe_Val;

		while ((Toe_Val > PTestThreshold) || (Toe_Val < NTestThreshold) && (repeatCount < 10)) {
			if (testAbort)
				break;
			repeatCount++;

			logWrite("repeatCount : " + repeatCount, false);
			logWrite("Toe Value : " + Toe_Val, false);
			int rotateCount = setRotationOption(Toe_Val);
			Adjust_Dir = checkAdjustDir(direction);
			switch (Adjust_Dir) {
			case Gripper_Rot_Up:
				logWrite("SETTING Adjust Dir - UP", true);
				break;
			case Gripper_Rot_Down:
				logWrite("SETTING Adjust Dir - DOWN", true);
				break;
			}
			if (Adjust_Dir == 0) {
				break;
			}
			logWrite(String.format("Rotation Count : %d", rotateCount), false);
			logWrite(String.format("Toe Value : %f", Toe_Val), false);

			// adjustToe(ADJ_LEFT, Adjust_Dir, rotateCount);
			adjustToe(Adjust_Dir, rotateCount);

			if (adjustFlag) {
				break;
			}
			if (noChangeFlag) {
				break;
			}

		}
		if (!testAbort) {
			if (direction == ADJ_PART_RIGHT) {
				if (isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold)) {
					logWrite("Current Toe Val : " + RToe_Val, true);
					logWrite("Adjust OK--------------------", true);
					IsAdjusted = true;
				} else {
					logWrite("Current Toe Val : " + RToe_Val, true);
					logWrite("Adjust Fail--------------------", true);
				}
			} else {
				if (isBetweenSpec(LToe_Val, NTestThreshold, PTestThreshold)) {
					logWrite("Current Toe Val : " + LToe_Val, true);
					logWrite("Adjust OK--------------------", true);
					IsAdjusted = true;
				} else {
					logWrite("Current Toe Val : " + LToe_Val, true);
					logWrite("Adjust Fail--------------------", true);
				}
			}
		} else
			IsAdjusted = true;
		adjustFlag = false;
		if (IsAdjusted) {
			CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
			ctim.parametrize(CartDOF.Z).setStiffness(3500);
			ctim.parametrize(CartDOF.X).setStiffness(2000);
			ctim.parametrize(CartDOF.Y).setStiffness(2000);
			ctim.parametrize(CartDOF.ROT).setStiffness(300);

			//schunk.GripperOpen();

			logWrite("Rot End, Return ROT Ready Pos", false);
			tcp.move(lin(Fr_Rot_Ready).setMode(ctim).setJointVelocityRel(0.6));

			for (int y = 0; y < 3; y++) {
				if (tightNutRunner()) {
					break;
				}
			}
		}
		return IsAdjusted;
	}

	/*****************************************************************
	 * 
	 * Common Adjust Function
	 * 
	 * @return
	 */

	// Set Adjust Count
	private int setRotationOption(double val) {
		int target = (int) (val / rotatecntoffset);
		if (target > 0) {
			return target;
		} else {
			target = 0;
		}
		return target;
		/*
		 * double target = 0.0; if (val > PTestThreshold) { target =
		 * Math.abs(val - PTestThreshold); } else if (val < NTestThreshold) {
		 * target = Math.abs(val - NTestThreshold); } else { target = 0; } //
		 * logWrite("Toe - Offset : " + Math.abs(val - ThresholdOffset), true);
		 * 
		 * // calculate how many times to rotate if (target > 0.4) { return 10;
		 * } else if (target > 0.35) { return 8; } else if (target > 0.3) {
		 * return 6; } else if (target > 0.25) { return 4; } else if (target >
		 * 0.15) { return 2; } else if (target > 0.05) { return 1; } else {
		 * return 0; }
		 */
	}

	// check Adjust Direction
	private int checkAdjustDir(int nDirection) {
		int Adjust_Dir = 0;

		// mapping the motion
		if (nDirection == ADJ_LEFT) {
			if (LToe_Val > PThreshold) {
				Adjust_Dir = Gripper_Rot_Up;
			}
			if (LToe_Val < NThreshold) {
				Adjust_Dir = Gripper_Rot_Down;
			}
		} else {
			if (RToe_Val > PThreshold) {
				Adjust_Dir = Gripper_Rot_Down;
			}
			if (RToe_Val < NThreshold) {
				Adjust_Dir = Gripper_Rot_Up;
			}
		}

		return Adjust_Dir;
	}

	// Adjust Toe
	/*
	 * private void adjustToe(int nDir, int nRotType, int nRotCount) { if
	 * (testAbort) return; if (nDir == ADJ_LEFT) { adjustToeLeft(nRotType,
	 * nRotCount); } else { adjustToeRight(nRotType, nRotCount); } }
	 */

	private void adjustToe(int nRotType, int nRotCount) {
		if (testAbort)
			return;
		CartesianImpedanceControlMode ctimRotation = new CartesianImpedanceControlMode();
		ctimRotation.parametrize(CartDOF.X).setStiffness(4500);
		ctimRotation.parametrize(CartDOF.Y).setStiffness(4500);
		ctimRotation.parametrize(CartDOF.Z).setStiffness(4500);
		ctimRotation.parametrize(CartDOF.ROT).setStiffness(300).setDamping(1.0);

		// to use Position Hold before Gripper Close
		CartesianImpedanceControlMode ctimPosHold = new CartesianImpedanceControlMode();
		ctimPosHold.parametrize(CartDOF.X).setStiffness(3000);
		ctimPosHold.parametrize(CartDOF.Y).setStiffness(3000);
		ctimPosHold.parametrize(CartDOF.Z).setStiffness(3000);
		ctimPosHold.parametrize(CartDOF.A).setStiffness(300);
		ctimPosHold.parametrize(CartDOF.B).setStiffness(200);
		ctimPosHold.parametrize(CartDOF.C).setStiffness(200);

		double prevToeVal = 0.0;

		ICallbackAction action = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
				while (!triggerInformation.getMotionContainer().isFinished()) {
					if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
						if (isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold)) {
							logWrite("Adjust Complete - Motion Cancel", true);
							triggerInformation.getMotionContainer().cancel();
							adjustFlag = true;
							return;
						}
					} else {
						if (isBetweenSpec(LToe_Val, NTestThreshold, PTestThreshold)) {
							logWrite("Adjust Complete - Motion Cancel", true);
							triggerInformation.getMotionContainer().cancel();
							adjustFlag = true;
							return;
						}
					}
					ThreadUtil.milliSleep(10);
				}
			}
		};

		if (nRotCount == 0) {
			logWrite("Last Toe Adjust Step", false);
			if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
				prevToeVal = RToe_Val;
			} else {
				prevToeVal = LToe_Val;
			}
			logWrite("Before Adjust Toe Val : " + prevToeVal, true);
			if (NoChangeCount >= 9) {
				logWrite("No Change Count : " + NoChangeCount, true);
				// No Change Count Reset
				NoChangeCount = 0;
				noChangeFlag = true;
				return;
				// if(!LooseNutRunner(NutRotate100))
				// return;
			}
			switch (nRotType) {
			case Gripper_Rot_Up:
				tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation).setJointVelocityRel(0.9));
				// Run before GripperClose
				tcp.moveAsync(positionHold(ctimPosHold, 100, TimeUnit.MILLISECONDS));
				// Gripper close and Move Frame Rot UP and Gripper Open
				//schunk.GripperClose();
				tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation).setJointVelocityRel(0.2).triggerWhen(new MotionPathCondition(ReferenceType.START, 0, 0), action));
				//schunk.GripperOpen();
				// increase Adjust Count
				totalAdjustCount = totalAdjustCount + 1;
				break;
			case Gripper_Rot_Down:
				tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation).setJointVelocityRel(0.9));
				tcp.moveAsync(positionHold(ctimPosHold, 100, TimeUnit.MILLISECONDS));
				//schunk.GripperClose();
				tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation).setJointVelocityRel(0.2).triggerWhen(new MotionPathCondition(ReferenceType.START, 0, 0), action));
				//schunk.GripperOpen();
				// increase Adjust Count
				totalAdjustCount = totalAdjustCount + 1;
				break;
			default:
				break;
			}
			// Wait for data to no change
			if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
				waitDataNoChange(ADJ_PART_RIGHT, NTestThreshold, PTestThreshold, adjustnochangedelay);
				if (checkDataNoChange(prevToeVal, RToe_Val)) {
					NoChangeCount++;
					logWrite("No Change Toe Value Count : " + NoChangeCount, true);
				} else {
					NoChangeCount = 0;
				}
			} else {
				waitDataNoChange(ADJ_PART_LEFT, NTestThreshold, PTestThreshold, adjustnochangedelay);
				if (checkDataNoChange(prevToeVal, LToe_Val)) {
					NoChangeCount++;
					logWrite("No Change Toe Value Count : " + NoChangeCount, true);
				} else {
					NoChangeCount = 0;
				}
			}
		} else {
			for (int i = 0; i < nRotCount; i++) {
				if (testAbort)
					return;
				try {
					if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
						prevToeVal = RToe_Val;
					} else {
						prevToeVal = LToe_Val;
					}

					logWrite("Current Rotation Count : " + (i + 1), true);
					logWrite("Before Adjus Toe Val : " + prevToeVal, true);

					if (NoChangeCount >= 9) {
						logWrite("No Change Count : " + NoChangeCount, true);
						// No Change Count Reset
						NoChangeCount = 0;
						noChangeFlag = true;
						return;
					}
					switch (nRotType) {
					case Gripper_Rot_Up:
						tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation).setJointVelocityRel(0.9));
						tcp.moveAsync(positionHold(ctimPosHold, 100, TimeUnit.MILLISECONDS));
						//schunk.GripperClose();
						tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation).setJointVelocityRel(0.9).triggerWhen(new MotionPathCondition(ReferenceType.START, 0, 0), action));
						//schunk.GripperOpen();
						// increase Adjust Count
						totalAdjustCount = totalAdjustCount + 1;
						break;
					case Gripper_Rot_Down:
						tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation).setJointVelocityRel(0.9));
						tcp.moveAsync(positionHold(ctimPosHold, 100, TimeUnit.MILLISECONDS));
						//schunk.GripperClose();
						tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation).setJointVelocityRel(0.9).triggerWhen(new MotionPathCondition(ReferenceType.START, 0, 0), action));
						//schunk.GripperOpen();
						// increase Adjust Count
						totalAdjustCount = totalAdjustCount + 1;
						break;
					default:
						break;
					}
					// Wait for data to no change
					if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
						if (isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold)) {
							break;
						}
						waitDataNoChange(ADJ_PART_RIGHT, NTestThreshold, PTestThreshold, adjustnochangedelay);
						if (checkDataNoChange(prevToeVal, RToe_Val)) {
							NoChangeCount++;
							logWrite("No Change Toe Value Count : " + NoChangeCount, true);
						} else {
							NoChangeCount = 0;
						}
					} else {
						if (isBetweenSpec(LToe_Val, NTestThreshold, PTestThreshold)) {
							break;
						}
						waitDataNoChange(ADJ_PART_LEFT, NTestThreshold, PTestThreshold, adjustnochangedelay);
						if (checkDataNoChange(prevToeVal, LToe_Val)) {
							NoChangeCount++;
							logWrite("No Change Toe Value Count : " + NoChangeCount, true);
						} else {
							NoChangeCount = 0;
						}
					}
				} catch (Exception ex) {
					logWrite(ex.toString(), true);
				}
			}
		}
		if (global.adjustPart == ADJ_PART_RIGHT) {
			while (IsValueChange_RH_Toe)
				ThreadUtil.milliSleep(10);
			if (isBetweenSpec(RToe_Val, NTestThreshold, PTestThreshold)) {
				adjustFlag = true;
			}
		} else {
			while (IsValueChange_LH_Toe)
				ThreadUtil.milliSleep(10);
			if (isBetweenSpec(LToe_Val, NTestThreshold, PTestThreshold)) {
				adjustFlag = true;
			}
		}
	}

	/*****************************************************************
	 * 
	 * Vision and Rotation TransFormation Function
	 * 
	 * @return
	 */

	private void SetRotTransformation() {
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(3500);
		ctim.parametrize(CartDOF.Y).setStiffness(3500);
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.ROT).setStiffness(200);

		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1, TimeUnit.MILLISECONDS));
		//schunk.GripperClose();

		ThreadUtil.milliSleep(GRIPPER_DELAY);

		if (global.mode == Mode_All) {
			if (MANUAL_ONLY_ADJ_PART == ADJ_PART_RIGHT) {
				Fr_Rot_Ready = Fr_Right_Rot_Ready;
				Fr_Rot_Up = Fr_Right_Rot_Up;
				Fr_Rot_Down = Fr_Right_Rot_Down;
			} else {
				Fr_Rot_Ready = Fr_Left_Rot_Ready;
				Fr_Rot_Up = Fr_Left_Rot_Up;
				Fr_Rot_Down = Fr_Left_Rot_Down;
			}
		}

		postP = lbr.getCurrentCartesianPosition(tcp);
		postP.setParent(Fr_Rot_Ready, true);
		// transformation = Transformation.ofTranslation(postP.getX(),
		// postP.getY(), postP.getZ());
		transformation = Transformation.ofDeg(postP.getX(), postP.getY(), postP.getZ(), 0, postP.getBetaRad(), postP.getGammaRad());

		Fr_Rot_Up.transform(Fr_Rot_Ready, transformation);
		Fr_Rot_Down.transform(Fr_Rot_Ready, transformation);

		hold.cancel();
		//schunk.GripperOpen();

	}

	private void visionTransform() {
		if (use_vision) {
			// transformL = Transformation.ofTranslation(0, global.visionLeftX,
			// 0);
			// transformR = Transformation.ofTranslation(0, global.visionRightX,
			// 0);
			transformL = Transformation.ofTranslation(0, global.visionLeftX, -global.visionLeftY);
			transformR = Transformation.ofTranslation(0, global.visionRightX, -global.visionRightY);
		} else {
			transformL = Transformation.ofTranslation(0, 0, 0);
			transformR = Transformation.ofTranslation(0, 0, 0);
		}

		logWrite("LEFT =" + transformL, use_vision);
		logWrite("RIGHT =" + transformR, use_vision);

		if (global.mode == Mode_All) {
			global.LeftToe_P3.transform(World.Current.getRootFrame(), transformL);
			global.LeftToe_P4.transform(World.Current.getRootFrame(), transformL);
			global.RightToe_P3.transform(World.Current.getRootFrame(), transformR);
			global.RightToe_P4.transform(World.Current.getRootFrame(), transformR);
		} else {
			if (global.mode == Mode_Left) {
				global.LeftToe_P3.transform(World.Current.getRootFrame(), transformL);
				global.LeftToe_P4.transform(World.Current.getRootFrame(), transformL);
			} else if (global.mode == Mode_Right) {
				global.RightToe_P3.transform(World.Current.getRootFrame(), transformR);
				global.RightToe_P4.transform(World.Current.getRootFrame(), transformR);
			}
		}
	}

	/*****************************************************************
	 * 
	 * Nut Runner Function
	 * 
	 * @return
	 */

	private boolean ToeLoose(int nLooseType) {
		boolean bResult = false;
		int cnt = 0;
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(GRIPPER_DELAY);

		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000, 3000, 3000, 3000, 3000, 3000, 10);

		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8));
		ThreadUtil.milliSleep(TOE_LOOSE_MOVE_DELAY);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8).setMode(jicm));
		ThreadUtil.milliSleep(TOE_LOOSE_MOVE_DELAY);

		try {
			switch (nLooseType) {
			case NUT_LOOSE:
				logWrite("Nut Loose", true);
				//nut.selectIO(NUT_LOOSE);
				mNutCommThread.onSendNutProgramSel(NUT_LOOSE);
				int cnts = 0;
				while(PSET_STATE == 2){
					if(cnts == 99){
						mNutCommThread.onSendNutProgramSel(NUT_LOOSE);
					}
					if(cnts > 199){
						cnts = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cnts++;
				}
				ThreadUtil.milliSleep(IO_SELECT_DELAY);
				break;
			case NUT_ROT_100:
				logWrite("Rot 100", true);
				//nut.selectIO(NUT_ROT_100);
				mNutCommThread.onSendNutProgramSel(NUT_ROT_100);
				int cntss = 0;
				while(PSET_STATE == 8){
					if(cntss == 99){
						mNutCommThread.onSendNutProgramSel(NUT_ROT_100);
					}
					if(cntss > 199){
						cntss = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cntss++;
				}
				ThreadUtil.milliSleep(IO_SELECT_DELAY);
				ThreadUtil.milliSleep(IO_SELECT_DELAY);
				break;
			}
			//nut.RunNutRunnerCW();
			TIGHTENING_STATE = 0;
			mNutCommThread.onSendNutStart();

			//while (!nut.CheckTightenOK()) {
			while (TIGHTENING_STATE != 0) {
				cnt++;
				if (TIGHTENING_STATE == 1) {
					logWrite("Tool Loose NOK", true);
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
					return false;
				}

				if (cnt > 500) {
					logWrite("Tool Loose Fail", true);
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
					return false;
				}
				ThreadUtil.milliSleep(10);
			}

			//nut.StopNutRunner();
			mNutCommThread.onSendNutStop();
			TIGHTENING_STATE = 0;
			logWrite("Tool Loose Complete - " + nLooseType, true);

			bResult = true;

		} catch (Exception ex) {
			getLogger().error(ex.toString());
		}

		return bResult;

	}

	/*private boolean ToeFastTight(int nType) {
		boolean bResult = false;
		int nCount = 0;
		int nDagi = 0;
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.Z).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8).setMode(ctim));
		ThreadUtil.milliSleep(50);
		try {
			logWrite("Fast Tight", true);
			//nut.selectIO(FAST_TIGHT);
			mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
			int cntss = 0;
			while(PSET_STATE == 6){
				if(cntss == 99){
					mNutCommThread.onSendNutProgramSel(FAST_TIGHT);
				}
				if(cntss > 199){
					cntss = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cntss++;
			}
			ThreadUtil.milliSleep(IO_SELECT_DELAY);

			tcp.move(linRel(0, 0, 10).setMode(ctim).setJointVelocityRel(0.5));

			int start = 1;
			while (start > 0) {
				if (nCount > 3) {
					logWrite("FastRun Count : " + nCount, true);
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					return false;
				}
				//nut.RunNutRunnerCW();
				TIGHTENING_STATE = 0;
				mNutCommThread.onSendNutStart();
				//while (!nut.CheckTightenNOK() && !nut.CheckTightenOK()) {
				while(TIGHTENING_STATE != 0){
					if (nDagi > 300) {
						nDagi = 0;
						return false;
					}
					nDagi++;
					ThreadUtil.milliSleep(10);
				}
				nDagi = 0;
				//nut.StopNutRunner();
				mNutCommThread.onSendNutStop();

				//if (nut.CheckTightenNOK()) {
				if (TIGHTENING_STATE == 1) {
					start = 0;
					break;
				} else
					nCount++;

				//while (nut.CheckTightenNOK() || nut.CheckTightenOK()) {
				while(TIGHTENING_STATE != 0){
					if (nDagi > 300) {
						nDagi = 0;
						return false;
					}
					nDagi++;
					ThreadUtil.milliSleep(10);
				}
				nDagi = 0;
			}

			// if (nut.CheckTightenNOK()) {
			// logWrite("Fast Run NOK", true);
			// nut.StopNutRunner();
			// }
			//nut.StopNutRunner();
			mNutCommThread.onSendNutStop();
			TIGHTENING_STATE = 0;
			logWrite("Tool Rotation 1300 Complete", true);
			bResult = true;
		} catch (Exception ex) {
			getLogger().error(ex.toString());
		}
		return bResult;
	}*/

	public boolean ToeTight(int nTightType) {
		boolean bTightResult = false;
		int cnt = 0;
		//nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);

		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.Z).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		switch (nTightType) {
		case NUT_TIGHT_50N:
			//nut.selectIO(NUT_TIGHT_50N);
			mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
			int cntss = 0;
			while(PSET_STATE == 4){
				if(cntss == 99){
					mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
				}
				if(cntss > 199){
					cntss = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cntss++;
			}
			break;
		/*case NUT_TIGHT_10N:
			//nut.selectIO(NUT_TIGHT_10N);
			mNutCommThread.onSendNutProgramSel(NUT_TIGHT_10N);
			int cnts = 0;
			while(PSET_STATE == 1){
				if(cnts == 99){
					mNutCommThread.onSendNutProgramSel(NUT_TIGHT_10N);
				}
				if(cnts > 199){
					cnts = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cnts++;
			}
			break;*/
		default:
			break;
		}
		ThreadUtil.milliSleep(IO_SELECT_DELAY);

		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.8));

		// IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1,
		// TimeUnit.MILLISECONDS));
		tcp.move(positionHold(ctim, 300, TimeUnit.MILLISECONDS));

		//nut.RunNutRunnerCW();
		TIGHTENING_STATE = 0;
		mNutCommThread.onSendNutStart();
		while (true) {
			cnt++;
			if (cnt > 1500) {
				logWrite("Tool Tight TimeOut Fail", true);
				break;
			} else {
				//if (nut.CheckTightenOK()) {
				if (TIGHTENING_STATE == 2) {
					bTightResult = true;
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					logWrite("Tool Tight Type : " + nTightType + " - Tight OK", true);
					TIGHTENING_STATE = 0;
					break;
				//} else if (nut.CheckTightenNOK()) {
				} else if (TIGHTENING_STATE == 1) {
					bTightResult = false;
					//nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
					logWrite("Tool Tight Type : " + nTightType + " - Tight NOK", true);
					break;
				}
			}
			ThreadUtil.milliSleep(10);
		}
		// hold.cancel();

		return bTightResult;
	}

	public void ConnectServer() {
		if (threadStart_main == THREAD_START) {
			thread_main = true;
			logWrite("Thread Start", false);
			r_main = new MainPRunnable();
			t_main = new Thread(r_main);
			t_main.start();
			
			
			while (!commOK_main) {
				ThreadUtil.milliSleep(100);
			}
		}
	}

	@Override
	public void dispose() {
		try {
			getLogger().info("nut end...");
			//nut.StopNutRunner();
			mNutCommThread.onSendNutStop();
			mNutCommThread.killComm();
			mNut_main.interrupt();
			whileMain = 0;
			//nut.IOClear();
			thread_main = false;
			client_main.endComm();
			if (use_graph)
				ftSender.interrupt();
			t_main.interrupt();
			
			//mNut_main = null;
			//mNutCommThread = null;
			getLogger().info("Stop Thread!");
			ThreadUtil.milliSleep(1000);
		} catch (Exception e) {
		}
		super.dispose();
	}

	// Runnable class for Packet
	static class MainPRunnable implements Runnable {

		private double prev_LHToe = 0.0;
		private int waitCountLHToe = 0;
		private double prev_RHToe = 0.0;
		private int waitCountRHToe = 0;

		public void ToeDataChangeCheck() {
			LhToeChangeVal = Math.abs(prev_LHToe - LToe_Val);
			RhToeChangeVal = Math.abs(prev_RHToe - RToe_Val);

			if (LhToeChangeVal > LTOE_CHANGE_THRESHOLD) {
				IsValueChange_LH_Toe = true;
				prev_LHToe = LToe_Val;
				waitCountLHToe = 0;
			} else {
				if (waitCountLHToe > 3) // 5
					IsValueChange_LH_Toe = false;
				else
					waitCountLHToe++;
			}

			if (RhToeChangeVal > RTOE_CHANGE_THRESHOLD) {
				IsValueChange_RH_Toe = true;
				prev_RHToe = RToe_Val;
				waitCountLHToe = 0;
			} else {
				if (waitCountRHToe > 3) // 5
					IsValueChange_RH_Toe = false;
				else
					waitCountRHToe++;
			}
		}

		public byte[] makeSendPacket() {
			byte[] send = new byte[SEND_MSG_SIZE];
			send[0] = (byte) 255; // HEADER
			send[1] = (byte) 170; // HEADER
			send[2] = global.carType; // cartype
			send[3] = global.adjustPart; // part
			send[4] = global.mode; // mode
			send[5] = global.workSignal; // working signal

			// logWrite(global.lhToe);
			System.arraycopy(global.DoubletoByte8_Array(global.lhToe), 0, send, 6, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.rhToe), 0, send, 14, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.lhCamber), 0, send, 22, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.rhCamber), 0, send, 30, 8);

			System.arraycopy(global.DoubletoByte8_Array(global.visionLeftX), 0, send, 38, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionLeftY), 0, send, 46, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionRightX), 0, send, 54, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionRightY), 0, send, 62, 8);

			send[70] = global.workState;

			return send;

		}

		@Override
		public void run() {
			client_main.startComm();
			byte[] msg = new byte[RECV_MSG_SIZE];
			SndMsg_main = new byte[SEND_MSG_SIZE];

			while (thread_main) {
				try {
					msg = client_main.receiveByteWait(msg.length);

					if (msg == null) {
						if (thread_main) {
							commOK_main = false;
							client_main.endComm();
							client_main.startComm();
							msg = new byte[RECV_MSG_SIZE];
							continue;
						}
					}

					commOK_main = true;

					global.Packet(msg);

					if (global.workSignal == 9)
						testAbort = true;
					else
						testAbort = false;

					if (global.workSignal == 100) {
						whileMain = 0;
						testAbort = true;
					}

					LToe_Val = global.lhToe;
					RToe_Val = global.rhToe;

					/* to debug */
					// dbToeLhVal = 1.7;

					LCamber_Val = global.lhCamber;
					RCamber_Val = global.rhCamber;

					ToeDataChangeCheck();

					SndMsg_main = makeSendPacket();

					client_main.sendByte(SndMsg_main);
				} catch (Exception ex) {

				}
			}

			System.out.println("<<<<< NCA Communication Thread Killed >>>>>");
			client_main.endComm();
		}
	}
	
	@Override
	public void cbToolResult(float torque, int tightState, float angle) {
		// TODO Auto-generated method stub
        logWrite("Last Toque Result:" + torque);
        logWrite("Last TightState Result:" + tightState);
        logWrite("Last Angle Result:" + angle);
        if (tightState == 1){
        	TIGHTENING_STATE = 2;
        	//TIGHTENING_NOK = false;
        }
        if (tightState == 0){
        	TIGHTENING_STATE = 1;
        	//TIGHTENING_NOK = true;
        }
	}

	@Override
	public void cbToolDinStateResult(int DinState) {
		// TODO Auto-generated method stub
		DIN_STATE = DinState;
	}

	@Override
	public void cbToolPSetResult(int Pset) {
		// TODO Auto-generated method stub
		System.out.println("PSet Update Ok " + Pset);
		PSET_STATE = Pset;
	};
}