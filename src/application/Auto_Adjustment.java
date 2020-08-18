package application;

import additionFunction.CallbackReceive;
import additionFunction.ForceTorqueDataSender;
import additionFunction.LogDataArray;
import additionFunction.NutCommThread;
import additionFunction.ForceTorqueDataSender.Type;
import additionFunction.Global;
import additionFunction.TCPClient;
import backgroundTask.*;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ThreadPoolExecutor.AbortPolicy;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;
import java.io.*;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.MotionCondition;
import com.kuka.roboticsAPI.conditionModel.MotionPathCondition;
import com.kuka.roboticsAPI.conditionModel.ReferenceType;
import com.kuka.roboticsAPI.conditionModel.TorqueComponentCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;

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
import com.sun.xml.internal.fastinfoset.algorithm.BuiltInEncodingAlgorithm.WordListener;

import ioTool.*;

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
public class Auto_Adjustment extends RoboticsAPIApplication implements
		CallbackReceive {
	@Inject
	private LBR lbr;

	@Inject
	private Controller kUKA_Sunrise_Cabinet_1;

	@Inject
	@Named("ToolTemplate")
	private Tool tool;

	private ObjectFrame tcp;

	// public static NutRunner nut;
	//public static ATC_Schunk schunk;

	NutCommThread mNutCommThread;
	private Thread mNut_main;
	private static int TIGHTENING_STATE = 0; // 0=NONE,1=NOK,2=OK
	private static int DIN_STATE = 0; // 0=NONE,1=NOK,2=OK
	private static int PSET_STATE = 255; // 0~N= ProgramNo 255=ISNONE
	public static ArrayList<LogDataArray> LogData_Arr = new ArrayList<LogDataArray>();
	private static double FinalToeTorqueData = 0.0;
	private static double FinalCamberData = 0.0;
	private static double ToeTorque = 0.0;
	private static double Angle = 0.0;
	static LogDataArray globalLogDataArray;
	private BufferedWriter fileWriter;
	public static double TOTAL_DEGREE;
	public static double TORQUE_VAL;


	public static ArrayList<Double> Arr_LToe_Val = new ArrayList<Double>();
	public static ArrayList<Double> Arr_RToe_Val = new ArrayList<Double>();
	public static ArrayList<Double> Arr_LCamber_Val = new ArrayList<Double>();
	public static ArrayList<Double> Arr_RCamber_Val = new ArrayList<Double>();
	public static double[] GLOBAL_MOV_LTOE = new double[30];
	public static double[] GLOBAL_MOV_RTOE = new double[30];

	private boolean reAdjustment = false;

	private static int RETRY_ADJ_CNT = 0;

	private static double BEFORETIGHT50N = 0;
	private static double AFTERTIGHT50N = 0;
	
	private int port_atlas;
	private String ip_atlas;

	boolean bInsertResult = false;

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

	private final int IO_SELECT_DELAY = 500;
	private final int GRIPPER_DELAY = 500;
	private final int NUT_STOP_DELAY = 500;
	private final int TOE_LOOSE_MOVE_DELAY = 100;
	private final int INSERT_NUT_SEEK_DELAY = 100;

	public Frame postP;
	// Left or Right Mode Use
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

	private double[] MAXThreshold = new double[12];
	private double[] MINThreshold = new double[12];

	private double MAXTestThreshold, MINTestThreshold;

	public static boolean IsValueChange_LH_Toe = true;
	public static boolean IsValueChange_RH_Toe = true;
	public static double LhToeChangeVal = 0.0;
	public static double RhToeChangeVal = 0.0;

	// Adjust Part
	private final int ADJ_PART_LEFT = 1;
	private final int ADJ_PART_RIGHT = 2;

	public final int Mode_Dissable = 0;
	public final int Mode_Left = 1;
	public final int Mode_Right = 2;
	public final int Mode_All = 5;

	public int rotateCount = 0;
	public int NoChangeCount = 0;
	public int Rot_100_Setting_Count = 0;
	public static double ToeChangeVal = 0.0;
	private int totalAdjustCount = 0;

	public final byte SIGNAL_RESET = (byte) 0;
	public final byte SIGNAL_MOVE_VISION_POS = (byte) 1;
	public final byte SIGNAL_MEASURE_VISION_DATA = (byte) 2;
	public final byte SIGNAL_LOOSE_NUT = (byte) 3;
	public final byte SIGNAL_READY_POS = (byte) 6;
	public final byte SIGNAL_TEST_HOME = (byte) 9;
	public final byte SIGNAL_TOOL_FAIL = (byte) 21;
	public final byte SIGNAL_CAMBER_TOOL = (byte) 41;
	public final byte SIGNAL_CAMBER_ADJ = (byte) 42;
	public final byte SIGNAL_CAMBER_OUT = (byte) 43;
	public final byte SIGNAL_CAMBER_WAITTOOL = (byte) 44;
	public final byte SIGNAL_CAMBER_WAITADJ = (byte) 45;
	public final byte SIGNAL_TOE_TOOL = (byte) 51;
	public final byte SIGNAL_TOE_ADJ = (byte) 52;
	public final byte SIGNAL_TOE_TIGHT = (byte) 53;
	public final byte SIGNAL_TOE_WAITTOOL = (byte) 54;
	public final byte SIGNAL_TOE_WAITADJ = (byte) 55;
	public final byte SIGNAL_TOE_LOOSE_NUT = (byte) 56;
	public final byte SIGNAL_TOE_OUT = (byte) 57;
	public final byte SIGNAL_ADJ_FAIL = (byte) 61;
	public static final byte SIGNAL_EXIT = (byte) 100;

	public final byte STATE_RESET = (byte) 0;
	public final byte STATE_VISION_POS = (byte) 1;
	public final byte STATE_MEASURE_VISION_DATA = (byte) 2;
	public final byte STATE_LOOSE_COMPLETE = (byte) 3;

	public final byte STATE_READY_POS = (byte) 6;
	public final byte STATE_TEST_HOME = (byte) 9;

	public final byte STATE_TOOL_FAIL = (byte) 21;
	public final byte STATE_CAMBER_TOOL = (byte) 41;
	public final byte STATE_CAMBER_ADJ = (byte) 42;
	public final byte STATE_CAMBER_WAITTOOL = (byte) 44;
	public final byte STATE_CAMBER_WAITADJ = (byte) 45;
	public final byte STATE_TOE_TOOL = (byte) 51;
	public final byte STATE_TOE_ADJ = (byte) 52;
	public final byte STATE_TOE_TIGHT = (byte) 53;
	public final byte STATE_TOE_WAITTOOL = (byte) 54;
	public final byte STATE_TOE_WAITADJ = (byte) 55;
	public final byte STATE_TOE_LOOSE_NUT = (byte) 56;
	public final byte STATE_TOE_OUT = (byte) 57;

	public final byte STATE_ADJ_FAIL = (byte) 61;

	private boolean adjustFlag = false;
	private boolean noChangeFlag = false;

	public static int whileMain = 0;

	public Frame changeFrame;

	public double waitChangeMin = 0;
	public double waitChangeMax = 0;

	ForceTorqueDataSender ftSender;

	private boolean toolToeRetry = false;

	public static boolean testAbort = false;
	public boolean specIn = false;
	private List<Frame> homeFrame;
	private int statusRobot = 0;

	private boolean useLog = true;
	private boolean useFTight = false;
	private int adjustToolType = 0;
	private double MinSpecOffset = 0;
	private double MaxSpecOffset = 0;
	private static boolean bTriggerCheckFail = false;

	private String totalToeValue;
	private String totalCamberValue;

	

	@Override
	public void initialize() {

		tcp = tool.getFrame("/tcp");
		tool.attachTo(lbr.getFlange());

		exio = new ExternalIOGroup(kUKA_Sunrise_Cabinet_1);

		//schunk = new ATC_Schunk(kUKA_Sunrise_Cabinet_1);

		// nut = new NutRunner(kUKA_Sunrise_Cabinet_1);

		global = new Global();

		initValue();

		initSpecValue();
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

		totalToeValue = "";
		TIGHTENING_STATE = 0;

		LogData_Arr.clear();
		globalLogDataArray = new LogDataArray();
		FinalToeTorqueData = 0.0;
		FinalCamberData = 0.0;
		ToeTorque = 0;
		Angle = 0;
		TIGHTENING_STATE = 0;
		Arr_LToe_Val.clear();
		Arr_RToe_Val.clear();
		Arr_LCamber_Val.clear();
		Arr_RCamber_Val.clear();
	}

	public void initSpecValue() {
		// QX
		MAXThreshold[0] = -0.04;
		MINThreshold[0] = -0.07;
		// PDM
		MAXThreshold[1] = -0.02;
		MINThreshold[1] = -0.05;
		// PDC
		MAXThreshold[2] = -0.02;
		MINThreshold[2] = -0.05;

		MAXThreshold[3] = -0.04;
		MINThreshold[3] = -0.07;

		// AEHEV
		MAXThreshold[4] = -0.02;
		MINThreshold[4] = -0.05;

		// AEEV
		MAXThreshold[5] = -0.02;
		MINThreshold[5] = -0.05;

		MAXThreshold[6] = -0.04;
		MINThreshold[6] = -0.07;

		MAXThreshold[7] = -0.04;
		MINThreshold[7] = -0.07;

		// AD
		MAXThreshold[8] = -0.02;
		MINThreshold[8] = -0.05;

		// ADS
		MAXThreshold[9] = -0.02;
		MINThreshold[9] = -0.05;

		MAXThreshold[10] = -0.04;
		MINThreshold[10] = -0.07;

		MAXThreshold[11] = -0.04;
		MINThreshold[11] = -0.07;

	}

	public void initProcessData() {
		use_main = getApplicationData().getProcessData("Use_Main").getValue();
		port_main = getApplicationData().getProcessData("Port_Main").getValue();
		ip_main = getApplicationData().getProcessData("IP_Main").getValue();

		port_atlas = getApplicationData().getProcessData("Port_Atlas")
				.getValue();
		ip_atlas = getApplicationData().getProcessData("IP_Atlas").getValue();

		use_adjust = getApplicationData().getProcessData("AdjustEnable")
				.getValue();
		use_vision = getApplicationData().getProcessData("VisionEnable")
				.getValue();
		use_tight50N = getApplicationData().getProcessData("Tight50NEnable")
				.getValue();
		adjustnochangedelay = getApplicationData().getProcessData(
				"AdjustNoChangeDelay").getValue();
		use_graph = getApplicationData().getProcessData("use_graph").getValue();
		rotatecntoffset = getApplicationData()
				.getProcessData("RotateCntOffset").getValue();

		statusRobot = getApplicationData().getProcessData("statusRobot")
				.getValue();

		useLog = getApplicationData().getProcessData("useLog").getValue();
		useFTight = getApplicationData().getProcessData("useFTight").getValue();
		adjustToolType = getApplicationData().getProcessData("AdjustToolType")
				.getValue();
		MinSpecOffset = getApplicationData().getProcessData("MinSpecOffset")
				.getValue();
		MaxSpecOffset = getApplicationData().getProcessData("MaxSpecOffset")
				.getValue();
	}

	public void initTool() {

		logWrite("Init Tool");

		// nut.moveNutRunnerHomePos();
		mNutCommThread.HomePos();

		//schunk.GripperOpen();

	}

	public void initFrame(String direction) {
		ObjectFrame tmp;

		String sFrame = "/P" + global.carType;

		if (direction.indexOf("Left") >= 0) {
			logWrite("<---- Load Left Frame ----> CarType : " + global.carType,
					false);

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
			logWrite(
					"<---- Load Right Frame ----> CarType : " + global.carType,
					false);
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
			logWrite("<---- Load All Frame ----> CarType : " + global.carType,
					false);
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
		long localstartTime = 0;
		long localendTime = 0;
		double localcycleTime = 0;
		localstartTime = System.currentTimeMillis();

		if (global.adjustPart == ADJ_PART_RIGHT) {
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
		localendTime = System.currentTimeMillis();
		localcycleTime = (Math.abs(localstartTime - localendTime) / 1000);

		String strFormat = String.format("%.2f", localcycleTime);

		logWrite("ToolIn Cycle Time : " + strFormat + " Sec", false);
		return RESULT;
	}

	private void nutLoose(boolean reloose) {

		if (global.adjustPart == ADJ_PART_RIGHT) {
			logWrite("Loose Nut", false);
			if (!ToeLoose(NUT_LOOSE)) {
				logWrite("Tool Position Error", true);
				global.workState = STATE_TOOL_FAIL;
			} else {
				// if (toeReLoose())
				if (!reloose) {
					global.workState = STATE_LOOSE_COMPLETE;
				}

				// else
				// global.workState = STATE_TOOL_FAIL;

				// if (Math.abs(global.F_RH_Toe_Count) > 0) {
				// int Adjust_Dir = 0;
				// if (global.F_RH_Toe_Count >= 0) {
				// Adjust_Dir = Gripper_Rot_Down;
				// } else {
				// Adjust_Dir = Gripper_Rot_Up;
				// }
				// Pre_adjustToe(Adjust_Dir, Math.abs(global.F_RH_Toe_Count));
				// }
			}
		} else {
			logWrite("Loose Nut", false);
			if (!ToeLoose(NUT_LOOSE)) {
				logWrite("Tool Position Error", true);
				global.workState = STATE_TOOL_FAIL;
			} else {
				// if (Math.abs(global.F_LH_Toe_Count) > 0) {
				// int Adjust_Dir = 0;
				// if (global.F_LH_Toe_Count >= 0) {
				// Adjust_Dir = Gripper_Rot_Up;
				// } else {
				// Adjust_Dir = Gripper_Rot_Down;
				// }
				// Pre_adjustToe(Adjust_Dir, Math.abs(global.F_LH_Toe_Count));
				// }

				// if (toeReLoose())
				if (!reloose) {
					global.workState = STATE_LOOSE_COMPLETE;
				}
				// else
				// global.workState = STATE_TOOL_FAIL;

			}
		}

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

	private boolean goReadyposition() {
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

			if (global.adjustPart == ADJ_PART_RIGHT) {
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
					tcp.move(lin(homeFrame.get(i)).setMode(ImpedanceMode1)
							.setJointVelocityRel(0.3));
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
					tcp.move(lin(homeFrame.get(i)).setMode(ImpedanceMode1)
							.setJointVelocityRel(0.3));
				}
			}
		}

		tcp.move(lin(global.Home).setJointVelocityRel(0.3));
		return true;
	}

	private boolean tightNutRunner() {
		boolean tight = false;
		int tightSelect;
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);
		long localstartTime = 0;
		long localendTime = 0;
		double localcycleTime = 0;
		localstartTime = System.currentTimeMillis();

		if (!testAbort) {
			// if (useFTight) {
			// ToeFastTight();
			// }

			// if (use_tight50N)
			tightSelect = NUT_TIGHT_50N;
			// else
			// tightSelect = NUT_TIGHT_10N;

			if (global.adjustPart == ADJ_PART_RIGHT) {
				totalToeValue += String.format("%.3f,", RToe_Val);
			} else {
				totalToeValue += String.format("%.3f,", LToe_Val);
			}

			// fastTightAngle();

			if (ToeTight(tightSelect)) {
				logWrite("Tool Tight Type : " + tightSelect + " OK", false);
				tight = true;
			} else {
				logWrite("Tool Tight Type : " + tightSelect + " NOK", true);
				tight = false;
			}

			if (global.adjustPart == ADJ_PART_RIGHT) {
				totalToeValue += String.format("%.3f,", RToe_Val);
			} else {
				totalToeValue += String.format("%.3f,", LToe_Val);
			}
		}

		localendTime = System.currentTimeMillis();
		localcycleTime = (Math.abs(localstartTime - localendTime) / 1000);

		String strFormat = String.format("%.2f", localcycleTime);

		logWrite("tightNutRunner Cycle Time : " + strFormat + " Sec", false);
		return tight;
	}

	/*
	 * private boolean ToeFastTight() { boolean bResult = false; int nCount = 0;
	 * int nDagi = 0; nut.StopNutRunner();
	 * ThreadUtil.milliSleep(NUT_STOP_DELAY); CartesianImpedanceControlMode ctim
	 * = new CartesianImpedanceControlMode();
	 * ctim.parametrize(CartDOF.X).setStiffness(2000);
	 * ctim.parametrize(CartDOF.Y).setStiffness(2000);
	 * ctim.parametrize(CartDOF.Z).setStiffness(2000);
	 * ctim.parametrize(CartDOF.ROT).setStiffness(300);
	 * tcp.move(lin(lbr.getCurrentCartesianPosition
	 * (tcp)).setJointVelocityRel(1.0).setMode(ctim));
	 * ThreadUtil.milliSleep(50); try { logWrite("Fast Tight", true);
	 * nut.selectIO(FAST_TIGHT); ThreadUtil.milliSleep(IO_SELECT_DELAY);
	 * 
	 * tcp.move(linRel(0, 0, 10).setMode(ctim).setJointVelocityRel(0.5));
	 * 
	 * int start = 1; while (start > 0) { if (nCount > 3) {
	 * logWrite("FastRun Count : " + nCount, true); nut.StopNutRunner(); return
	 * false; } nut.RunNutRunnerCW(); while (!nut.CheckTightenNOK() &&
	 * !nut.CheckTightenOK()) { if (nDagi > 300) { nDagi = 0; return false; }
	 * nDagi++; ThreadUtil.milliSleep(10); } nDagi = 0; nut.StopNutRunner();
	 * 
	 * if (nut.CheckTightenNOK()) { start = 0; break; } else nCount++;
	 * 
	 * while (nut.CheckTightenNOK() || nut.CheckTightenOK()) { if (nDagi > 300)
	 * { nDagi = 0; return false; } nDagi++; ThreadUtil.milliSleep(10); } nDagi
	 * = 0; }
	 * 
	 * nut.StopNutRunner(); logWrite("Tool Rotation 1300 Complete", true);
	 * bResult = true; } catch (Exception ex) {
	 * getLogger().error(ex.toString()); } return bResult; }
	 */

	private void toeAdjust() {
		long localstartTime = 0;
		long localendTime = 0;
		double localcycleTime = 0;
		localstartTime = System.currentTimeMillis();

		boolean testOk = false;
		calcThreshold();
		logWrite("Toe Align", false);

		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		if (use_adjust) {
			if (!specIn) {
				SetRotTransformation();

				if (global.adjustPart == ADJ_PART_RIGHT) {
					if (!toeTest(ADJ_PART_RIGHT)) {
						logWrite("Toe Right Adjust Error", true);
						global.workState = STATE_ADJ_FAIL;
					} else {
						if (!testAbort) {
							testOk = true;
						}
					}
				} else if (global.adjustPart == ADJ_PART_LEFT) {
					if (!toeTest(ADJ_PART_LEFT)) {
						logWrite("Toe Left Adjust Error", true);
						global.workState = STATE_ADJ_FAIL;
					} else {
						if (!testAbort) {
							testOk = true;
						}
					}

				} else if (global.adjustPart == 0) {
					logWrite("Toe Right Adjust Error", true);
					global.workState = STATE_ADJ_FAIL;
				}

			} else {
				testOk = true;
			}

			if (testOk) {
				//schunk.GripperOpen();

				logWrite("Rot End, Return ROT Ready Pos", false);
				tcp.move(lin(Fr_Rot_Ready).setMode(ctim).setJointVelocityRel(
						0.6));

				// for (int y = 0; y < 3; y++) {
				// if (tightNutRunner()) {
				// break;
				// }
				// }
				if (!reAdjustment) {
					if (global.adjustPart == ADJ_PART_RIGHT) {
						BEFORETIGHT50N = RToe_Val;
					} else {
						BEFORETIGHT50N = LToe_Val;
					}
					logWrite("BEFORETIGHT50N : " + BEFORETIGHT50N, false);
				}
				// if (tightNutRunner()) {
				// boolean result = reworkTest();
				// if (result) {
				// global.workState = STATE_TOE_ADJ;
				// } else
				// global.workState = STATE_ADJ_FAIL;
				// } else
				// logWrite("Tight Error", true);
				global.workState = SIGNAL_TOE_ADJ;

			}

		} else {
			logWrite("use Adjust select False", true);
			global.workState = STATE_ADJ_FAIL;
		}
		localendTime = System.currentTimeMillis();
		localcycleTime = (Math.abs(localstartTime - localendTime) / 1000);

		String strFormat = String.format("%.2f", localcycleTime);

		logWrite("ToeAdjust Cycle Time : " + strFormat + " Sec", false);
	}

	private boolean reworkTest() {
		ThreadUtil.milliSleep(3000);
		if (!reAdjustment) {
			if (global.adjustPart == ADJ_PART_RIGHT) {
				AFTERTIGHT50N = RToe_Val;
			} else {
				AFTERTIGHT50N = LToe_Val;
			}
			logWrite("AFTERTIGHT50N : " + AFTERTIGHT50N, false);
		}
		int result = checkSpecData();
		boolean testOk = false;
		if (RETRY_ADJ_CNT > 1) {
			testOk = false;
			logWrite("Over Retrying", true);
			return testOk;
		}
		if (result == 1) {
			if (ToeLoose(NUT_LOOSE)) {
				ThreadUtil.milliSleep(2000);
				if (tightNutRunner()) {
					ThreadUtil.milliSleep(2000);
				} else
					logWrite("rework Tight Error", true);

				testOk = true;
			}
		} else if (result == 2) {
			if (ToeLoose(NUT_LOOSE)) {
				ThreadUtil.milliSleep(2000);
				testOk = false;
			} else
				logWrite("Rework Loose Error", true);
		} else
			testOk = true;

		RETRY_ADJ_CNT++;
		return testOk;
	}

	private int checkSpecData() { // 0:OK 1:Min 2:Max
		int specOk = 0;
		double Toe_Val = 0;

		if (global.adjustPart == ADJ_PART_RIGHT) {
			while (IsValueChange_RH_Toe)
				ThreadUtil.milliSleep(10);
		} else {
			while (IsValueChange_LH_Toe)
				ThreadUtil.milliSleep(10);
		}
		if (global.adjustPart == ADJ_PART_RIGHT) {
			Toe_Val = RToe_Val;
		} else {
			Toe_Val = LToe_Val;
		}
		logWrite("checkspecdata ==" + Toe_Val);
		if (Toe_Val < -0.04) {
			specOk = 1;
		} else if (Toe_Val > 0.04) {
			specOk = 2;
		} else
			specOk = 0;
		return specOk;
	}

	private void statusRobotSave(int status) {
		getApplicationData().getProcessData("statusRobot").setValue(status);
	}

	private void saveResult(String cycleTime){
		SimpleDateFormat dateformat = new SimpleDateFormat("yyyy-MM-dd");
		SimpleDateFormat timeformat = new SimpleDateFormat("HH:mm:ss");
		Date date = new Date();
		Date time = new Date();
		String strdate = dateformat.format(date);
		String strtime = dateformat.format(time);
		String DirectoryBasePath = "C:\\KRC\\ApplicationServer\\logs\\Tasks\\";
		String[] DirectoryDatePath = {"","",""};
		String FilePath = strtime.replace(":","");
		try {
			DirectoryDatePath = strdate.split("-");
			File Directory = new File(DirectoryBasePath + DirectoryDatePath[0]);
			if(!Directory.exists()){
				try{
					Directory.mkdir();
				}catch(Exception e){
					e.getStackTrace();
				}
			}
			Directory = new File(DirectoryBasePath + DirectoryDatePath[0] + "\\" + DirectoryDatePath[1]);
			if(!Directory.exists()){
				try{
					Directory.mkdir();
				}catch(Exception e){
					e.getStackTrace();
				}
			}
			Directory = new File(DirectoryBasePath + DirectoryDatePath[0] + "\\" + DirectoryDatePath[1] + 
					"\\" + DirectoryDatePath[2]);
			if(!Directory.exists()){
				try{
					Directory.mkdir();
				}catch(Exception e){
					e.getStackTrace();
				}
			}
			
			BufferedWriter fileSave = new BufferedWriter(new FileWriter(DirectoryBasePath + DirectoryDatePath[0] 
					+ "\\" + DirectoryDatePath[1] + "\\" + DirectoryDatePath[2] + "\\" +  FilePath +".dat"));
			fileSave.write("[Result]");
			fileSave.newLine();
			fileSave.write("Date=" + date + " " + time);
			fileSave.newLine();
			fileSave.write("BarCode=" + global.Barcode);
			fileSave.newLine();
			fileSave.write("CarType=" + global.carType);
			fileSave.newLine();
			fileSave.write("TotalLapTime=" + cycleTime);
			fileSave.newLine();
			fileSave.write("Result=OK");
			fileSave.newLine();
			
			ArrayList<Double> Arr_Toe_Val = new ArrayList<Double>();
			ArrayList<Double> Arr_Camber_Val = new ArrayList<Double>();
			if (global.adjustPart == ADJ_PART_RIGHT) {
				Arr_Toe_Val = Arr_RToe_Val;
				Arr_Camber_Val = Arr_RCamber_Val;
			}else{
				Arr_Toe_Val = Arr_RToe_Val;
				Arr_Camber_Val = Arr_RCamber_Val;
			}
			double FirstToeData = 0.0;
			double FirstCamberData = 0.0;
			for (double toeVal : Arr_Toe_Val) {
				if(toeVal != 0){
					FirstToeData = toeVal;
				}
			}
			for (double camberVal : Arr_Camber_Val) {
				if(camberVal != 0){
					FirstCamberData = camberVal;
				}
			}
			
			String[] ToeVals = totalToeValue.split(",");
			
			fileSave.write("FirstToeData="+ToeVals[0]);
			fileSave.newLine();
			fileSave.write("FirstCamberData="+FirstCamberData);
			fileSave.newLine();
			fileSave.write("FinalToeData="+ToeVals[3]);
			fileSave.newLine();
			fileSave.write("FinalCamberData="+Arr_Camber_Val.get(Arr_Camber_Val.size()));
			fileSave.newLine();
			
			fileSave.write("Angle=" + TOTAL_DEGREE);
			fileSave.newLine();
			
			fileSave.write("CamberTorque=" + FinalCamberData);
			fileSave.newLine();
			fileSave.write("ToeTorque=" + FinalToeTorqueData);
			fileSave.newLine();
			
			fileSave.write("Data=" +  LogData_Arr.size());
			fileSave.newLine();
			
			for(int i = 0 ; i < LogData_Arr.size() ; i ++){
				//000=1.111,2.222,3.333,4 //toe,camber,angle(조정시 ),status
				fileSave.write(String.format("%3d", i)  + "=" + LogData_Arr.get(i).TOEData + ", "
						+ LogData_Arr.get(i).CamberData + ", " + LogData_Arr.get(i).TorqueData + ", " 
						+ LogData_Arr.get(i).AngleData + ", " + LogData_Arr.get(i).StatusData);
				//이 위치 데이터 기록
				fileSave.newLine();
			}
			
			fileSave.close();
		}catch(Exception ex){
			System.out.println(ex.toString());
		}
	}

	@Override
	public void run() {

		if (use_graph) {
			ftSender = new ForceTorqueDataSender(lbr, tcp, 30000, -1, 20,
					Type.JOINT_TORQUE, getLogger());
			ftSender.start();
		}
		// your application execution starts here
		long start = 0;
		long end = 0;
		double cycleTime = 0;
		String strFormat = "";

		int preWorksignal = -1;
		whileMain = 1;

		initFrame("Left");

		while (whileMain > 0) {

			if (preWorksignal == global.workSignal)
				continue;
			preWorksignal = global.workSignal;

			switch (global.workSignal) {
			case SIGNAL_RESET: // test Ready
				statusRobotSave(STATE_RESET);

				logWrite("Reset", false);
				if (global.mode == Mode_Left) {
					initFrame("Left");
				} else if (global.mode == Mode_Right) {
					initFrame("Right");
				} else if (global.mode == Mode_All) {
					initFrame("All");
				} else {
					initFrame("Right");
				}

				start = 0;
				end = 0;
				cycleTime = 0.0;
				totalAdjustCount = 0;
				adjustFlag = false;
				noChangeFlag = false;
				specIn = false;
				toolToeRetry = false;
				totalToeValue = "";
				reAdjustment = false;
				RETRY_ADJ_CNT = 0;
				BEFORETIGHT50N = 0;
				AFTERTIGHT50N = 0;
				Date today = new Date();
				SimpleDateFormat date = new SimpleDateFormat("yyyyMMddhhmmss");

				try {
					if (global.adjustPart == ADJ_PART_RIGHT) {
						fileWriter = new BufferedWriter(new FileWriter(
								"C:\\KRC\\ApplicationServer\\logs\\Tasks\\RightDegree"
										+ date.format(today) + ".log"));
					} else {
						fileWriter = new BufferedWriter(new FileWriter(
								"C:\\KRC\\ApplicationServer\\logs\\Tasks\\LeftDegree"
										+ date.format(today) + ".log"));
					}
				} catch (IOException e) {
					logWrite("Degree Log File Create Error", true);
					e.printStackTrace();
				}
				initTool();

				global.workState = STATE_RESET;

				break;

			case SIGNAL_MOVE_VISION_POS: // move vision position
				statusRobotSave(SIGNAL_MOVE_VISION_POS);
				start = System.currentTimeMillis();
				// nut.selectIO(NUT_INSERT_15N);
				mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
				int cnt = 0;
				while (PSET_STATE == 1) {
					if (cnt == 99) {
						mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
					}
					if (cnt > 199) {
						cnt = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cnt++;
				}

				specIn = false;
				if (global.adjustPart == ADJ_PART_LEFT) {
					tcp.move(lin(global.LeftToe_P1).setJointVelocityRel(0.5));
				} else if (global.adjustPart == ADJ_PART_RIGHT) {
					tcp.move(lin(global.RightToe_P1).setJointVelocityRel(0.5));
				} else {
					tcp.move(lin(global.RightToe_P1).setJointVelocityRel(0.5));
				}

				global.workState = STATE_VISION_POS;

				break;

			case SIGNAL_MEASURE_VISION_DATA: // Measure vision position
				visionTransform();
				// nut.selectIO(NUT_INSERT_15N);
				mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
				int cnts = 0;
				while (PSET_STATE == 1) {
					if (cnts == 99) {
						mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
					}
					if (cnts > 199) {
						cnts = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cnts++;
				}
				if (toeToolIn()) {
					statusRobotSave(SIGNAL_MEASURE_VISION_DATA);
					global.workState = STATE_MEASURE_VISION_DATA;
				}
				break;

			case SIGNAL_LOOSE_NUT: // Loose nut position
				statusRobotSave(SIGNAL_LOOSE_NUT);
				if (use_adjust) {
					nutLoose(false);
				} else
					global.workState = STATE_LOOSE_COMPLETE;
				break;
			case SIGNAL_CAMBER_TOOL:
				statusRobotSave(SIGNAL_CAMBER_TOOL);
				camberToolIn();
				break;

			case SIGNAL_CAMBER_ADJ:
				statusRobotSave(SIGNAL_CAMBER_ADJ);
				camberAdjust();
				break;

			case SIGNAL_TOE_TOOL:
				statusRobotSave(SIGNAL_TOE_TOOL);
				global.workState = STATE_TOE_TOOL;
				break;

			case SIGNAL_TOE_ADJ:
				statusRobotSave(SIGNAL_TOE_ADJ);
				if (use_adjust)
					toeAdjust();
				else
					global.workState = SIGNAL_TOE_ADJ;
				break;

			case SIGNAL_TOE_TIGHT:
				statusRobotSave(SIGNAL_TOE_TIGHT);
				tightNutRunner();
				if (use_adjust) {
					global.workState = STATE_TOE_TIGHT;
				} else {
					global.workState = STATE_READY_POS;
				}
				break;

			case SIGNAL_CAMBER_OUT:
				statusRobotSave(STATE_READY_POS);
				global.workState = STATE_READY_POS;
				break;

			case SIGNAL_TOE_OUT:
				statusRobotSave(SIGNAL_TOE_OUT);
				if (global.adjustPart == ADJ_PART_RIGHT) {
					toeOut(ADJ_PART_RIGHT);
				} else {
					toeOut(ADJ_PART_LEFT);
				}

				statusRobotSave(STATE_READY_POS);
				global.workState = STATE_READY_POS;

				logWrite("Adjust End");

				if (global.adjustPart == ADJ_PART_RIGHT) {
					totalToeValue += String.format("%.3f,", RToe_Val);
					logWrite("Last Toe : " + RToe_Val
							+ " 3 Seconds Ago AVG_Toe: " + GLOBAL_MOV_RTOE[29],
							true);
				} else {
					totalToeValue += String.format("%.3f,", LToe_Val);
					logWrite("Last Toe : " + LToe_Val
							+ " 3 Seconds Ago AVG_Toe: " + GLOBAL_MOV_LTOE[29],
							true);
				}

				break;

			case SIGNAL_CAMBER_WAITTOOL:
				global.workState = STATE_CAMBER_WAITTOOL;
				break;
			case SIGNAL_TOE_WAITTOOL:
				global.workState = STATE_TOE_WAITTOOL;
				break;
			case SIGNAL_CAMBER_WAITADJ:
				global.workState = STATE_CAMBER_WAITADJ;
				break;
			case SIGNAL_TOE_WAITADJ:
				reAdjustment = true;
				global.workState = STATE_TOE_WAITADJ;
				break;

			case SIGNAL_TOE_LOOSE_NUT:
				statusRobotSave(SIGNAL_TOE_LOOSE_NUT);
				if (use_adjust) {
					nutLoose(true);
				}
				global.workState = STATE_TOE_LOOSE_NUT;
				break;

			case SIGNAL_READY_POS:
				statusRobotSave(STATE_READY_POS);
				global.workState = STATE_READY_POS;
				break;

			case SIGNAL_TEST_HOME:
				statusRobot = getApplicationData()
						.getProcessData("statusRobot").getValue();
				logWrite("----------TEST HOME statusRobot set to ["
						+ statusRobot + "]");
				if (statusRobot == SIGNAL_MOVE_VISION_POS
						|| statusRobot == STATE_READY_POS
						|| statusRobot == STATE_RESET) {
					goReadyposition();
				} else {
					if (global.adjustPart == ADJ_PART_LEFT) {
						toeOut(ADJ_PART_LEFT);
					} else if (global.adjustPart == ADJ_PART_RIGHT) {
						toeOut(ADJ_PART_RIGHT);
					} else
						toeOut(ADJ_PART_RIGHT);
				}
				global.workState = STATE_TEST_HOME;

				end = System.currentTimeMillis();

				logWrite("First,BeforeTight,AfterTight,End," + global.carType,
						true);
				logWrite(totalToeValue, true);

				cycleTime = (Math.abs(start - end) / 1000);

				strFormat = String.format("%.2f", cycleTime);

				logWrite("Total Cycle Time : " + strFormat + " Sec", false);
				logWrite("Total Adjust Count : " + totalAdjustCount, false);

				saveResult(strFormat);
				try {
					fileWriter.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					logWrite("Degree Log File Close Error", true);
					e.printStackTrace();
				}
				break;
			default:
				break;
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

	// 추후 사용
	/*private void GripperClose() {
		exio.setOUT15(false);
		exio.setOUT16(true);
	}

	private void GripperOpen() {
		exio.setOUT15(true);
		exio.setOUT16(false);
	}

	private void GripperClose_IOCheck() {
		GripperClose();

		BooleanIOCondition gripC = new BooleanIOCondition(
				exio.getInput("IN15"), true);
		ICondition cond = gripC;
		while (!this.getObserverManager().waitFor(cond, GRIPPER_DELAY,
				TimeUnit.MILLISECONDS)) {
			GripperClose();
			ThreadUtil.milliSleep(10);
		}
	}

	private void GripperOpen_IOCheck() {
		GripperOpen();

		BooleanIOCondition gripC = new BooleanIOCondition(
				exio.getInput("IN16"), true);
		ICondition cond = gripC;
		while (!this.getObserverManager().waitFor(cond, GRIPPER_DELAY,
				TimeUnit.MILLISECONDS)) {
			GripperOpen();
			ThreadUtil.milliSleep(10);
		}
	}

	private void GripperHold() {
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(3500);
		ctim.parametrize(CartDOF.Y).setStiffness(3500);
		ctim.parametrize(CartDOF.Z).setStiffness(3500);
		ctim.parametrize(CartDOF.ROT).setStiffness(200);

		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1,
				TimeUnit.MILLISECONDS));
		schunk.GripperClose();

		ThreadUtil.milliSleep(GRIPPER_DELAY); // 500
		// Mode Change Position Control
		hold.cancel();
		schunk.GripperOpen();
		// tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(0.1));
		System.out.println("Gripper Hold Function end");
	}*/

	public void calcThreshold() {
		int selSpec = 0;
		double AdjustToeSpecOffset = 0;
		if (global.carType > 0)
			selSpec = global.carType - 1;
		MinSpecOffset = getApplicationData().getProcessData("MinSpecOffset")
				.getValue();
		MaxSpecOffset = getApplicationData().getProcessData("MaxSpecOffset")
				.getValue();
		if (global.adjustPart == ADJ_PART_RIGHT) {
			MAXTestThreshold = MAXThreshold[selSpec] + MaxSpecOffset;
			MINTestThreshold = MINThreshold[selSpec] + MinSpecOffset;
		} else {
			MAXTestThreshold = MAXThreshold[selSpec] + MaxSpecOffset;
			MINTestThreshold = MINThreshold[selSpec] + MinSpecOffset;
		}

		if (!reAdjustment) {
			if (global.adjustPart == ADJ_PART_LEFT) {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("LToe_Val:" + LToe_Val, true);
				specIn = isBetweenSpec(LToe_Val, -0.04, 0.04);
			} else if (global.adjustPart == ADJ_PART_RIGHT) {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("RToe_Val:" + RToe_Val, true);
				specIn = isBetweenSpec(RToe_Val, -0.04, 0.04);
			} else {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("RToe_Val:" + RToe_Val, true);
				specIn = isBetweenSpec(RToe_Val, -0.04, 0.04);
			}

			logWrite("PThreshold : " + MAXTestThreshold, true);
			logWrite("NThreshold : " + MINTestThreshold, true);
		} else {
			if (global.adjustPart == ADJ_PART_LEFT) {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("LToe_Val:" + LToe_Val, true);
				specIn = isBetweenSpec(LToe_Val, MINTestThreshold,
						MAXTestThreshold);
			} else if (global.adjustPart == ADJ_PART_RIGHT) {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("RToe_Val:" + RToe_Val, true);
				specIn = isBetweenSpec(RToe_Val, MINTestThreshold,
						MAXTestThreshold);
			} else {
				// logWrite("Spec In No Loose Nut", true);
				logWrite("RToe_Val:" + RToe_Val, true);
				specIn = isBetweenSpec(RToe_Val, MINTestThreshold,
						MAXTestThreshold);
			}

			logWrite("PThreshold : " + MAXTestThreshold, true);
			logWrite("NThreshold : " + MINTestThreshold, true);

			AdjustToeSpecOffset = BEFORETIGHT50N - AFTERTIGHT50N;
			logWrite("ReAdjustThreshold BEFORETIGHT50N: " + BEFORETIGHT50N,
					true);
			logWrite("ReAdjustThreshold AFTERTIGHT50N: " + AFTERTIGHT50N, true);
			logWrite("ReAdjustThreshold AdjustToeSpecOffset: "
					+ AdjustToeSpecOffset, true);
			if (BEFORETIGHT50N < 0 && AFTERTIGHT50N > 0
					&& Math.abs(AdjustToeSpecOffset) > 0.1) {
				// MAXTestThreshold -= AdjustToeSpecOffset;
				MINTestThreshold -= (Math.abs(AdjustToeSpecOffset) / 2);
				MAXTestThreshold -= (Math.abs(AdjustToeSpecOffset) / 2);// MINTestThreshold
																		// +
																		// 0.05;
				logWrite("ReAdjustThreshold MINTestThreshold: "
						+ AdjustToeSpecOffset, true);
				logWrite("ReAdjustThreshold MAXTestThreshold: "
						+ AdjustToeSpecOffset, true);
			}
		}
	}

	private boolean checkDataNoChange(double prevData, double curData) {
		double ToeValDif;

		ToeValDif = Math.abs(prevData - curData);
		logWrite("Toe Diff : " + ToeValDif, true);

		return ToeValDif < 0.003;
	}

	private void waitDataNoChange(int nDir, double min, double max, int delay) {
		double val;

		if (nDir == ADJ_PART_RIGHT) {
			val = RToe_Val;
		} else {
			val = LToe_Val;
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
		else {
			if (useLog)
				getLogger().info(log);
		}
	}

	private void logWrite(String log) {
		if (useLog) {
			getLogger().info(log);
		}
	}

	/*****************************************************************
	 * 
	 * Move Home Position
	 * 
	 * @return
	 */

	public void MoveHomePosition() {
		tcp.move(lin(global.Home).setJointVelocityRel(0.6));
		logWrite("Move Home");

		if (checkHome()) {
			logWrite("Home - Complete");

		}
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

		// nut.selectIO(NUT_INSERT_15N);
		/*mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
		int cnt = 0;
		while (PSET_STATE == 1) {
			if (cnt == 99) {
				mNutCommThread.onSendNutProgramSel(NUT_INSERT_15N);
			}
			if (cnt > 199) {
				cnt = 0;
				break;
			}
			ThreadUtil.milliSleep(10);
			cnt++;
		}*/
		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(4000);
		ctim.parametrize(CartDOF.Y).setStiffness(3000);
		ctim.parametrize(CartDOF.Z).setStiffness(4000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		CartesianSineImpedanceControlMode csim = new CartesianSineImpedanceControlMode();
		csim.parametrize(CartDOF.X).setStiffness(2800);
		csim.parametrize(CartDOF.Y).setStiffness(2800).setAmplitude(10)
				.setFrequency(4);
		csim.parametrize(CartDOF.Z).setStiffness(4000);
		csim.parametrize(CartDOF.ROT).setStiffness(300);

		// Move Frame Insert Ready Position
		tcp.moveAsync(lin(p2).setJointVelocityRel(0.6).setBlendingRel(0.3));
		logWrite("Move P2 Approach");
		tcp.move(lin(p3).setJointVelocityRel(0.6));
		logWrite("Move Approach p3");

		// Move to TieRoad
		double currentFX = lbr.getExternalForceTorque(tcp).getForce().getX();
		ForceComponentCondition fcx = new ForceComponentCondition(tcp,
				CoordinateAxis.X, currentFX - 20, currentFX + 20);

		IMotionContainer mc = tcp.move(lin(p4).setJointVelocityRel(0.1)
				.setMode(ctim).breakWhen(fcx));

		if (mc.hasFired(fcx)) {
			logWrite("Move P4(TieRod) BREAKED!!" + fcx, true);
			bInserted = true;
		}
		logWrite("Trying oscilation!!");
		ForceComponentCondition fcx2 = new ForceComponentCondition(tcp,
				CoordinateAxis.X, currentFX - 35, currentFX + 35);
		IMotionContainer mc2 = tcp.move(linRel(-70.0, 0.0, 0.0)
				.setJointVelocityRel(0.1).setMode(ctim).breakWhen(fcx2));

		if (mc2.hasFired(fcx2)) {
			logWrite("Move X Direction (TieRod) BREAKED!!" + fcx2, true);
			bInserted = true;
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
					// nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
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

		distance = Math
				.abs(lbr.getCurrentCartesianPosition(tcp, target).getZ());
		logWrite("Distance : " + distance, false);

		return distance;
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
		logWrite("Move p4 TieRoad Inner");
		bInserted = true;
		//GripperHold();

		// Detect Nut
		if (DetectTighteningNut()) {
			// Check Inserted
			bInserted = checkNutInserted(direction);
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
			double currentForceZ = lbr.getExternalForceTorque(tcp).getForce()
					.getZ();

			ForceComponentCondition fcZ = new ForceComponentCondition(tcp,
					CoordinateAxis.Z, currentForceZ - 35.0,
					currentForceZ + 35.0);

			mc = tcp.move(linRel(0.0, 0.0, 50.0).setJointVelocityRel(0.4)
					.breakWhen(fcZ).setMode(csim)); 

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
		// nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		// TIGHTENING_OK = false;
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		CartesianSineImpedanceControlMode sineMode = new CartesianSineImpedanceControlMode();
		sineMode.parametrize(CartDOF.X).setStiffness(2800).setAmplitude(10)
				.setFrequency(2).setDamping(1.0);
		sineMode.parametrize(CartDOF.Y).setStiffness(2800).setAmplitude(10)
				.setFrequency(2).setDamping(1.0);
		sineMode.parametrize(CartDOF.Z).setStiffness(2000).setBias(20)
				.setDamping(1.0);
		sineMode.parametrize(CartDOF.A).setStiffness(300);
		sineMode.parametrize(CartDOF.B).setStiffness(300);
		sineMode.parametrize(CartDOF.C).setStiffness(300);

		CartesianSineImpedanceControlMode sineMode2 = new CartesianSineImpedanceControlMode();
		sineMode2.parametrize(CartDOF.X).setStiffness(3000);
		sineMode2.parametrize(CartDOF.Y).setStiffness(3000);
		sineMode2.parametrize(CartDOF.Z).setStiffness(2000).setBias(20);
		// TODO : sk.kim 190506_1327
		sineMode2.parametrize(CartDOF.A).setStiffness(/* 300 */100)
				.setAmplitude(/* 5 */3).setFrequency(1);
		sineMode2.parametrize(CartDOF.B).setStiffness(300);
		sineMode2.parametrize(CartDOF.C).setStiffness(200);

		try {
			bInsertResult = false;

			logWrite("Start Insert Nut Runner");

			ICallbackAction action = new ICallbackAction() {
				@Override
				public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
					while (!triggerInformation.getMotionContainer()
							.isFinished()) {

						switch (TIGHTENING_STATE) {
						case 1:
							bInsertResult = false;
							// mNutCommThread.onSendNutStop();
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
						/*
						 * //if (nut.CheckTightenOK()) { if (TIGHTENING_OK) {
						 * bInsertResult = true; bTriggerCheckFail = true;
						 * mNutCommThread.onSendNutStop(); TIGHTENING_OK =
						 * false; logWrite("Nut Insert - OK", false);
						 * triggerInformation.getMotionContainer().cancel();
						 * break; } else if (!TIGHTENING_OK) { bInsertResult =
						 * false; mNutCommThread.onSendNutStop(); TIGHTENING_OK
						 * = false; logWrite("Nut Insert - NOK", true);
						 * triggerInformation.getMotionContainer().cancel();
						 * break; }
						 */
					}
				}
			};

			// nut.RunNutRunnerCW();
			TIGHTENING_STATE = 0;
			mNutCommThread.onSendNutStart();

			tcp.move(linRel(0.0, 0.0, 20.0)
					.setMode(sineMode)
					.setJointVelocityRel(0.1)
					.triggerWhen(
							new MotionPathCondition(ReferenceType.START, 0, 0),
							action));
		} catch (Exception ex) {
			logWrite(ex.toString(), true);
		}

		while (!bTriggerCheckFail) {
			cnt++;
			if (cnt > 500) {
				logWrite("Tight Signal Read Fail : InsertNutRunner", true);
				return false;
			}

			switch (TIGHTENING_STATE) {
			case 1:
				bInsertResult = false;
				// mNutCommThread.onSendNutStop();
				// TIGHTENING_STATE = 0;
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
			 * //if (nut.CheckTightenOK()) { if(TIGHTENING_OK){ bInsertResult =
			 * true; //nut.StopNutRunner(); mNutCommThread.onSendNutStop();
			 * TIGHTENING_OK = false; logWrite("Nut Trigger After Insert - OK",
			 * false); bTriggerCheckFail = true; } else if (!TIGHTENING_OK) {
			 * bInsertResult = false; mNutCommThread.onSendNutStop();
			 * TIGHTENING_OK = false; bTriggerCheckFail = true;
			 * logWrite("Nut Trigger After Insert - NOK", true); }
			 */
			ThreadUtil.milliSleep(10);
		}

		mNutCommThread.onSendNutStop();
		TIGHTENING_STATE = 0;
		// TIGHTENING_OK = false;

		tcp.move(linRel(0.0, 0.0, 20.0).setMode(sineMode2).setCartVelocity(20));
		return bInsertResult;
	}

	public boolean toeOut(int direction) {
		long localstartTime = 0;
		long localendTime = 0;
		double localcycleTime = 0;
		localstartTime = System.currentTimeMillis();

		boolean bResult = false;
		double dbRot = 0;
		if (direction == ADJ_PART_RIGHT) {
			dbRot = 2.0;
		} else {
			dbRot = -2.0; // ToolOut Rot Angle
		}

		ToeLoose(NUT_ROT_100);

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
		csim.parametrize(CartDOF.C).setStiffness(300).setAmplitude(10)
				.setFrequency(2);

		//GripperHold();
		Frame rotationP1;
		Frame rotationP2;
		Frame target;

		rotationP1 = lbr.getCurrentCartesianPosition(tcp);
		rotationP2 = rotationP1.copyWithRedundancy().transform(tcp,
				Transformation.ofDeg(0, 0, 0, dbRot, 0, 0));
		target = rotationP1.copyWithRedundancy().transform(tcp,
				Transformation.ofTranslation(0, 0, -20));
		tcp.move(lin(rotationP2).setJointVelocityRel(0.5).setMode(ctim));
		logWrite("Move Rot A : " + dbRot, false);
		tcp.move(lin(target).setJointVelocityRel(0.4).setMode(csim));
		logWrite("Move P4 Approach");
		tcp.move(lin(target).setJointVelocityRel(0.4).setMode(ctim1));
		logWrite("Move P4 Approach");
		tcp.move(lin(target).setJointVelocityRel(0.4));
		logWrite("Move P4 Approach");
		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000,
				3000, 3000, 3000, 3000, 200, 10);
		IMotionContainer hold = tcp.moveAsync(positionHold(jicm, -1,
				TimeUnit.MILLISECONDS));

		boolean check = false;
		//schunk.GripperClose();
		for (int i = 0; i < 10; i++) {
			// check = nut.moveNutRunnerHomePos();
			check = mNutCommThread.HomePos();
			if (check) {
				if (exio.getIN14())
					break;
				else
					check = false;
			}
		}
		mNutCommThread.onSendNutStop();
		TIGHTENING_STATE = 0;
		//schunk.GripperOpen();
		ThreadUtil.milliSleep(GRIPPER_DELAY);
		hold.cancel();

		if (exio.getIN14()) {
			logWrite("Nut Runner Home - OK");
			moveOutPosition(direction);
			MoveHomePosition();
			logWrite("Tool Out Success");
			bResult = true;
		} else {
			// nut.StopNutRunner();
			logWrite("Nut Runner Home - Fail", true);
		}

		localendTime = System.currentTimeMillis();
		localcycleTime = (Math.abs(localstartTime - localendTime) / 1000);

		String strFormat = String.format("%.2f", localcycleTime);

		logWrite("toeOut Cycle Time : " + strFormat + " Sec", false);
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
		csim.parametrize(CartDOF.Y).setStiffness(5000).setAmplitude(10)
				.setFrequency(4);
		csim.parametrize(CartDOF.Z).setStiffness(5000);
		csim.parametrize(CartDOF.A).setStiffness(100);
		csim.parametrize(CartDOF.B).setStiffness(300);
		csim.parametrize(CartDOF.C).setStiffness(300);

		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000,
				3000, 3000, 3000, 3000, 3000, 10);

		double currentFX = lbr.getExternalForceTorque(tcp).getForce().getX();
		ForceComponentCondition fcx = new ForceComponentCondition(tcp,
				CoordinateAxis.X, currentFX - 30, currentFX + 30);

		IMotionContainer mc = tcp.move(linRel(50, 0, 0)
				.setJointVelocityRel(0.1).setMode(csim).breakWhen(fcx));
		logWrite("Move P3 Insert Ready");
		if (mc.hasFired(fcx)) {
			logWrite("Detected Colision 15Nm", true);
			// tcp.move(linRel(0, 1.0, 0.0).setJointVelocityRel(0.1));
			tcp.move(linRel(30, 0, 0).setJointVelocityRel(0.1).setMode(jicm));
		}
		tcp.moveAsync(lin(p2).setJointVelocityRel(0.6).setBlendingCart(5));
		logWrite("Move P2 Approach");
		tcp.move(lin(p1).setJointVelocityRel(0.6));
		logWrite("Move P1 Approach");

	}

	private boolean toeTest(int direction) {
		double Toe_Val = 0;
		int repeatCount;
		boolean IsAdjusted;
		int Adjust_Dir = 0;

		if (direction == ADJ_PART_RIGHT) {
			Toe_Val = RToe_Val;
		} else {
			Toe_Val = LToe_Val;
		}

		totalToeValue = String.format("%.3f,", Toe_Val);

		repeatCount = 0;
		IsAdjusted = false;
		noChangeFlag = false;

		while ((Toe_Val > MAXTestThreshold) || (Toe_Val < MINTestThreshold)
				&& (repeatCount < 5)) {
			if (testAbort)
				break;
			repeatCount++;
			if (direction == ADJ_PART_RIGHT) {
				Toe_Val = RToe_Val;
			} else
				Toe_Val = LToe_Val;

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

			adjustToe(Adjust_Dir, rotateCount);
			if (rotateCount == 1) {
				ThreadUtil.milliSleep(3000);
			}

			if (adjustFlag) {
				break;
			}
			if (noChangeFlag) {
				break;
			}
		}

		if (!testAbort) {
			if (direction == ADJ_PART_RIGHT) {
				if (isBetweenSpec(RToe_Val, MINTestThreshold, MAXTestThreshold)) {
					logWrite("Adjust OK--------------------", true);
					IsAdjusted = true;
				} else {
					logWrite("Adjust Fail--------------------", true);
				}
				logWrite("Current Toe Val : " + RToe_Val, true);
			} else {
				if (isBetweenSpec(LToe_Val, MINTestThreshold, MAXTestThreshold)) {
					logWrite("Adjust OK--------------------", true);
					IsAdjusted = true;
				} else {
					logWrite("Adjust Fail--------------------", true);
				}
				logWrite("Current Toe Val : " + LToe_Val, true);
			}
		} else
			IsAdjusted = false;
		adjustFlag = false;

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
		int target = (int) ((val - MINTestThreshold) / rotatecntoffset);

		return Math.abs(target);
	}

	// check Adjust Direction
	private int checkAdjustDir(int nDirection) {
		int Adjust_Dir = 0;

		// mapping the motion
		if (nDirection == ADJ_PART_RIGHT) {
			if (RToe_Val > MAXTestThreshold) {
				Adjust_Dir = Gripper_Rot_Down;
			} else if (RToe_Val < MINTestThreshold) {
				Adjust_Dir = Gripper_Rot_Up;
			}
		} else {
			if (LToe_Val > MAXTestThreshold) {
				Adjust_Dir = Gripper_Rot_Up;
			} else if (LToe_Val < MINTestThreshold) {
				Adjust_Dir = Gripper_Rot_Down;
			}
		}

		return Adjust_Dir;
	}

	double Past_Degree = 0.0;
	double Post_Degree = 0.0;

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
					if (global.adjustPart == ADJ_PART_RIGHT) {
						if (isBetweenSpec(RToe_Val, MINTestThreshold,
								MAXTestThreshold)) {
							logWrite("Adjust Complete - Motion Cancel");
							triggerInformation.getMotionContainer().cancel();
							// adjustFlag = true;
							return;
						}
					} else {
						if (isBetweenSpec(LToe_Val, MINTestThreshold,
								MAXTestThreshold)) {
							logWrite("Adjust Complete - Motion Cancel");
							triggerInformation.getMotionContainer().cancel();
							// adjustFlag = true;
							return;
						}
					}
					ThreadUtil.milliSleep(10);
				}
			}
		};

		if (nRotCount == 0) {
			logWrite("Last Toe Adjust Step", false);
			if (global.adjustPart == ADJ_PART_RIGHT) {
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
			}
			switch (nRotType) {
			case Gripper_Rot_Up:
				if (adjustToolType == 0) {
					tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation)
							.setJointVelocityRel(0.8));
					// Run before GripperClose
					tcp.moveAsync(positionHold(ctimPosHold, 100,
							TimeUnit.MILLISECONDS));
					// Gripper close and Move Frame Rot UP and Gripper Open
					//schunk.GripperClose();
					tcp.move(lin(Fr_Rot_Up)
							.setMode(ctimRotation)
							.setJointVelocityRel(0.1)
							.triggerWhen(
									new MotionPathCondition(
											ReferenceType.START, 0, 0), action));
					//schunk.GripperOpen();
					double Post_Degree = Math.toDegrees(lbr
							.getCurrentCartesianPosition(tcp, Fr_Rot_Down)
							.getAlphaRad());
					logWrite("Adjust Degree: " + Post_Degree);
					TOTAL_DEGREE += Post_Degree;
					
					
					
					try {
						if (global.adjustPart == ADJ_PART_RIGHT) {
							globalLogDataArray.AngleData = Post_Degree;
							globalLogDataArray.TOEData = RToe_Val;
							globalLogDataArray.CamberData = RCamber_Val;
							globalLogDataArray.StatusData = global.workState;
							LogData_Arr.add(globalLogDataArray);
							
							fileWriter.write("Adjust Degree, " + Post_Degree
									+ ", RTOEVAL, " + RToe_Val);
						} else {
							globalLogDataArray.AngleData = Post_Degree;
							globalLogDataArray.TOEData = LToe_Val;
							globalLogDataArray.CamberData = LCamber_Val;
							globalLogDataArray.StatusData = global.workState;
							LogData_Arr.add(globalLogDataArray);
							
							fileWriter.write("Adjust Degree, " + Post_Degree
									+ ", LTOEVAL, " + LToe_Val);
						}
						fileWriter.newLine();

					} catch (IOException e) {
						// TODO Auto-generated catch block
						logWrite("Degree Log File Write Error", true);
						e.printStackTrace();
					}
				} else if (adjustToolType == 1) {
					logWrite("CW_RUN", true);
					//schunk.GripperClose();
					// nut.selectIO(CW_RUN);
					mNutCommThread.onSendNutProgramSel(CW_RUN);
					int cnt = 0;
					while (PSET_STATE == 3) {
						if (cnt == 99) {
							mNutCommThread.onSendNutProgramSel(CW_RUN);
						}
						if (cnt > 199) {
							cnt = 0;
							break;
						}
						ThreadUtil.milliSleep(10);
						cnt++;
					}
					ThreadUtil.milliSleep(IO_SELECT_DELAY);
					// nut.RunNutRunnerCW();
					mNutCommThread.onSendNutStart();
					ThreadUtil.milliSleep(500); // 10rpm 60degree/seconds -> 10
												// rpm halfseconds? 30?

					// nut.StopNutRunner();
					// schunk.GripperOpen();
				}
				// increase Adjust Count
				totalAdjustCount = totalAdjustCount + 1;
				break;
			case Gripper_Rot_Down:
				if (adjustToolType == 0) {
					tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation)
							.setJointVelocityRel(0.9));
					tcp.moveAsync(positionHold(ctimPosHold, 100,
							TimeUnit.MILLISECONDS));
					//schunk.GripperClose();
					tcp.move(lin(Fr_Rot_Down)
							.setMode(ctimRotation)
							.setJointVelocityRel(0.1)
							.triggerWhen(
									new MotionPathCondition(
											ReferenceType.START, 0, 0), action));
					//schunk.GripperOpen();
					double Post_Degree = Math.toDegrees(lbr
							.getCurrentCartesianPosition(tcp, Fr_Rot_Up)
							.getAlphaRad());
					logWrite("Adjust Degree: " + Post_Degree);
					TOTAL_DEGREE += Post_Degree;
					try {
						if (global.adjustPart == ADJ_PART_RIGHT) {
							globalLogDataArray.AngleData = Post_Degree;
							globalLogDataArray.TOEData = RToe_Val;
							globalLogDataArray.CamberData = RCamber_Val;
							globalLogDataArray.TorqueData = 0.0;
							globalLogDataArray.StatusData = global.workState;
							LogData_Arr.add(globalLogDataArray);
							fileWriter.write("Adjust Degree, " + Post_Degree
									+ ", RTOEVAL, " + RToe_Val);
						} else {
							globalLogDataArray.AngleData = Post_Degree;
							globalLogDataArray.TOEData = LToe_Val;
							globalLogDataArray.CamberData = LCamber_Val;
							globalLogDataArray.TorqueData = 0.0;
							globalLogDataArray.StatusData = global.workState;
							LogData_Arr.add(globalLogDataArray);
							fileWriter.write("Adjust Degree, " + Post_Degree
									+ ", LTOEVAL, " + LToe_Val);
						}
						fileWriter.newLine();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						logWrite("Degree Log File Write Error", true);
						e.printStackTrace();
					}
				} else if (adjustToolType == 1) {
					//schunk.GripperClose();
					logWrite("CCW_RUN", true);
					// nut.selectIO(CCW_RUN);
					mNutCommThread.onSendNutProgramSel(CCW_RUN);
					int cnt = 0;
					while (PSET_STATE == 5) {
						if (cnt == 99) {
							mNutCommThread.onSendNutProgramSel(CCW_RUN);
						}
						if (cnt > 199) {
							cnt = 0;
							break;
						}
						ThreadUtil.milliSleep(10);
						cnt++;
					}
					ThreadUtil.milliSleep(IO_SELECT_DELAY);
					// nut.RunNutRunnerCW();
					ThreadUtil.milliSleep(500); // 10rpm 60degree/seconds -> 10
												// rpm halfseconds? 30?
												// Convertunits.com
				}
				// increase Adjust Count
				totalAdjustCount = totalAdjustCount + 1;
				break;
			default:
				break;
			}
			if (global.adjustPart == ADJ_PART_RIGHT) {
				waitDataNoChange(ADJ_PART_RIGHT, MINTestThreshold - 0.02,
						MAXTestThreshold + 0.02, adjustnochangedelay);
				if (checkDataNoChange(prevToeVal, RToe_Val)) {
					// if (checkDataNoChange(GLOBAL_MOV_RTOE[9], RToe_Val)) {
					NoChangeCount++;
					logWrite("No Change Toe Value Count : " + NoChangeCount,
							true);
				} else {
					NoChangeCount = 0;
				}
			} else {
				waitDataNoChange(ADJ_PART_LEFT, MINTestThreshold - 0.02,
						MAXTestThreshold + 0.02, adjustnochangedelay);
				if (checkDataNoChange(prevToeVal, LToe_Val)) {
					// if (checkDataNoChange(GLOBAL_MOV_LTOE[9], LToe_Val)) {
					NoChangeCount++;
					logWrite("No Change Toe Value Count : " + NoChangeCount,
							true);
				} else {
					NoChangeCount = 0;
				}
			}
		} else {
			for (int i = 0; i < nRotCount; i++) {
				if (testAbort)
					return;
				try {
					if (global.adjustPart == ADJ_PART_RIGHT) {
						prevToeVal = RToe_Val;
					} else {
						prevToeVal = LToe_Val;
					}
					if (NoChangeCount >= 9) {
						logWrite("No Change Count : " + NoChangeCount, true);
						// No Change Count Reset
						NoChangeCount = 0;
						noChangeFlag = true;
						return;
					}
					switch (nRotType) {
					case Gripper_Rot_Up:
						if (adjustToolType == 0) {
							tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation)
									.setJointVelocityRel(0.9));
							tcp.moveAsync(positionHold(ctimPosHold, 100,
									TimeUnit.MILLISECONDS));
							//schunk.GripperClose();
							tcp.move(lin(Fr_Rot_Up)
									.setMode(ctimRotation)
									.setJointVelocityRel(0.9)
									.triggerWhen(
											new MotionPathCondition(
													ReferenceType.START, 0, 0),
											action));
							//schunk.GripperOpen();
							double Post_Degree = Math.toDegrees(lbr
									.getCurrentCartesianPosition(tcp,
											Fr_Rot_Down).getAlphaRad());
							logWrite("Adjust Degree: " + Post_Degree);
							TOTAL_DEGREE += Post_Degree;
							try {
								if (global.adjustPart == ADJ_PART_RIGHT) {
									globalLogDataArray.AngleData = Post_Degree;
									globalLogDataArray.TOEData = RToe_Val;
									globalLogDataArray.CamberData = RCamber_Val;
									globalLogDataArray.TorqueData = 0.0;
									globalLogDataArray.StatusData = global.workState;
									LogData_Arr.add(globalLogDataArray);
									fileWriter.write("Adjust Degree, "
											+ Post_Degree + ", RTOEVAL, "
											+ RToe_Val);
								} else {
									globalLogDataArray.AngleData = Post_Degree;
									globalLogDataArray.TOEData = LToe_Val;
									globalLogDataArray.CamberData = LCamber_Val;
									globalLogDataArray.TorqueData = 0.0;
									globalLogDataArray.StatusData = global.workState;
									LogData_Arr.add(globalLogDataArray);
									fileWriter.write("Adjust Degree, "
											+ Post_Degree + ", LTOEVAL, "
											+ LToe_Val);
								}
								fileWriter.newLine();
							} catch (IOException e) {
								// TODO Auto-generated catch block
								logWrite("Degree Log File Write Error", true);
								e.printStackTrace();
							}
						} else if (adjustToolType == 1) {
							//schunk.GripperClose();
							logWrite("CW_RUN", true);
							// nut.selectIO(CW_RUN);
							mNutCommThread.onSendNutProgramSel(CW_RUN);
							int cnt = 0;
							while (PSET_STATE == 3) {
								if (cnt == 99) {
									mNutCommThread.onSendNutProgramSel(CW_RUN);
								}
								if (cnt > 199) {
									cnt = 0;
									break;
								}
								ThreadUtil.milliSleep(10);
								cnt++;
							}
							ThreadUtil.milliSleep(IO_SELECT_DELAY);
							// nut.RunNutRunnerCW();
							// ThreadUtil.milliSleep(500); //10rpm
							// 60degree/seconds
							// 30degree/halfseconds
							ThreadUtil.milliSleep(500 * rotateCount);
						}
						// increase Adjust Count
						totalAdjustCount = totalAdjustCount + 1;
						break;
					case Gripper_Rot_Down:
						if (adjustToolType == 0) {
							tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation)
									.setJointVelocityRel(0.9));
							tcp.moveAsync(positionHold(ctimPosHold, 100,
									TimeUnit.MILLISECONDS));
							//schunk.GripperClose();
							tcp.move(lin(Fr_Rot_Down)
									.setMode(ctimRotation)
									.setJointVelocityRel(0.9)
									.triggerWhen(
											new MotionPathCondition(
													ReferenceType.START, 0, 0),
											action));
							//schunk.GripperOpen();
							double Post_Degree = Math
									.toDegrees(lbr.getCurrentCartesianPosition(
											tcp, Fr_Rot_Up).getAlphaRad());
							logWrite("Adjust Degree: " + Post_Degree);
							TOTAL_DEGREE += Post_Degree;
							try {
								if (global.adjustPart == ADJ_PART_RIGHT) {
									globalLogDataArray.AngleData = Post_Degree;
									globalLogDataArray.TOEData = RToe_Val;
									globalLogDataArray.CamberData = RCamber_Val;
									globalLogDataArray.TorqueData = 0.0;
									globalLogDataArray.StatusData = global.workState;
									LogData_Arr.add(globalLogDataArray);
									fileWriter.write("Adjust Degree, "
											+ Post_Degree + ", RTOEVAL, "
											+ RToe_Val);
								} else {
									globalLogDataArray.AngleData = Post_Degree;
									globalLogDataArray.TOEData = LToe_Val;
									globalLogDataArray.CamberData = LCamber_Val;
									globalLogDataArray.TorqueData = 0.0;
									globalLogDataArray.StatusData = global.workState;
									LogData_Arr.add(globalLogDataArray);
									fileWriter.write("Adjust Degree, "
											+ Post_Degree + ", LTOEVAL, "
											+ LToe_Val);
								}
								fileWriter.newLine();
							} catch (IOException e) {
								// TODO Auto-generated catch block
								logWrite("Degree Log File Write Error", true);
								e.printStackTrace();
							}
						} else if (adjustToolType == 1) {
							logWrite("CCW_RUN", true);
							//schunk.GripperClose();
							// nut.selectIO(CCW_RUN);
							mNutCommThread.onSendNutProgramSel(CCW_RUN);
							int cnt = 0;
							while (PSET_STATE == 5) {
								if (cnt == 99) {
									mNutCommThread.onSendNutProgramSel(CCW_RUN);
								}
								if (cnt > 199) {
									cnt = 0;
									break;
								}
								ThreadUtil.milliSleep(10);
								cnt++;
							}
							ThreadUtil.milliSleep(IO_SELECT_DELAY);
							// nut.RunNutRunnerCW();
							// ThreadUtil.milliSleep(500); //10rpm
							// 60degree/seconds
							// Convertunits.com
							ThreadUtil.milliSleep(500 * rotateCount);
						}
						// increase Adjust Count
						totalAdjustCount = totalAdjustCount + 1;
						break;
					default:
						break;
					}

					if (global.adjustPart == ADJ_PART_RIGHT) {
						if (isBetweenSpec(RToe_Val, MINTestThreshold,
								MAXTestThreshold)) {
							break;
						}
						waitDataNoChange(ADJ_PART_RIGHT,
								MINTestThreshold - 0.02,
								MAXTestThreshold + 0.02, adjustnochangedelay);
						if (checkDataNoChange(prevToeVal, RToe_Val)) {
							// if (checkDataNoChange(GLOBAL_MOV_RTOE[9],
							// RToe_Val)) {
							NoChangeCount++;
							logWrite("No Change Toe Value Count : "
									+ NoChangeCount, true);
						} else {
							NoChangeCount = 0;
						}
					} else {

						if (isBetweenSpec(LToe_Val, MINTestThreshold,
								MAXTestThreshold)) {

							break;
						}
						waitDataNoChange(ADJ_PART_LEFT,
								MINTestThreshold - 0.02,
								MAXTestThreshold + 0.02, adjustnochangedelay);
						if (checkDataNoChange(prevToeVal, LToe_Val)) {
							// if (checkDataNoChange(GLOBAL_MOV_LTOE[9],
							// LToe_Val)) {
							NoChangeCount++;
							logWrite("No Change Toe Value Count : "
									+ NoChangeCount, true);
						} else {
							NoChangeCount = 0;
						}
					}
				} catch (Exception ex) {
					logWrite(ex.toString(), true);
				}

			}
		}
		if (adjustToolType == 1) {
			mNutCommThread.onSendNutStop();
			if(global.adjustPart == ADJ_PART_RIGHT){
				globalLogDataArray.AngleData = Post_Degree;
				globalLogDataArray.TOEData = RToe_Val;
				globalLogDataArray.CamberData = RCamber_Val;
				globalLogDataArray.TorqueData = ToeTorque;
				globalLogDataArray.StatusData = global.workState;
				LogData_Arr.add(globalLogDataArray);
			}else{
				globalLogDataArray.AngleData = Post_Degree;
				globalLogDataArray.TOEData = LToe_Val;
				globalLogDataArray.CamberData = LCamber_Val;
				globalLogDataArray.TorqueData = ToeTorque;
				globalLogDataArray.StatusData = global.workState;
				LogData_Arr.add(globalLogDataArray);
			}
			TIGHTENING_STATE = 0;
			//schunk.GripperOpen();
		}
		if (global.adjustPart == ADJ_PART_RIGHT) {
			while (IsValueChange_RH_Toe)
				ThreadUtil.milliSleep(10);
			if (isBetweenSpec(RToe_Val, MINTestThreshold, MAXTestThreshold)) {
				adjustFlag = true;
			}
		} else {
			while (IsValueChange_LH_Toe)
				ThreadUtil.milliSleep(10);
			if (isBetweenSpec(LToe_Val, MINTestThreshold, MAXTestThreshold)) {
				adjustFlag = true;
			}
		}
	}

	private void Pre_adjustToe(int nRotType, int nRotCount) {
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

		for (int i = 0; i < nRotCount; i++) {
			if (testAbort)
				return;
			try {

				switch (nRotType) {
				case Gripper_Rot_Up:
					tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation)
							.setJointVelocityRel(0.9));
					tcp.moveAsync(positionHold(ctimPosHold, 100,
							TimeUnit.MILLISECONDS));
					//schunk.GripperClose();
					tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation)
							.setJointVelocityRel(0.9));
					//schunk.GripperOpen();
					// increase Adjust Count
					totalAdjustCount = totalAdjustCount + 1;

					break;
				case Gripper_Rot_Down:

					tcp.move(lin(Fr_Rot_Up).setMode(ctimRotation)
							.setJointVelocityRel(0.9));
					tcp.moveAsync(positionHold(ctimPosHold, 100,
							TimeUnit.MILLISECONDS));
					//schunk.GripperClose();
					tcp.move(lin(Fr_Rot_Down).setMode(ctimRotation)
							.setJointVelocityRel(0.9));
					//schunk.GripperOpen();
					// increase Adjust Count
					totalAdjustCount = totalAdjustCount + 1;

					break;
				default:
					break;
				}
			} catch (Exception ex) {
				logWrite(ex.toString(), true);
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

		IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1,
				TimeUnit.MILLISECONDS));
		//schunk.GripperClose();

		ThreadUtil.milliSleep(GRIPPER_DELAY);

		if (global.mode == Mode_All) {
			if (global.adjustPart == ADJ_PART_RIGHT) {
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
		transformation = Transformation.ofTranslation(postP.getX(),
				postP.getY(), postP.getZ());
		// transformation = Transformation.ofDeg(postP.getX(), postP.getY(),
		// postP.getZ(), 0, postP.getBetaRad(), postP.getGammaRad());
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
			transformL = Transformation.ofTranslation(0, global.visionLeftX,
					-global.visionLeftY);
			transformR = Transformation.ofTranslation(0, global.visionRightX,
					-global.visionRightY);
		} else {
			transformL = Transformation.ofTranslation(0, 0, 0);
			transformR = Transformation.ofTranslation(0, 0, 0);
		}

		logWrite("LEFT =" + transformL, use_vision);
		logWrite("RIGHT =" + transformR, use_vision);
		if (global.mode == Mode_All) {
			global.LeftToe_P3.transform(World.Current.getRootFrame(),
					transformL);
			global.LeftToe_P4.transform(World.Current.getRootFrame(),
					transformL);
			global.RightToe_P3.transform(World.Current.getRootFrame(),
					transformR);
			global.RightToe_P4.transform(World.Current.getRootFrame(),
					transformR);
		} else {
			if (global.mode == Mode_Left) {
				global.LeftToe_P3.transform(World.Current.getRootFrame(),
						transformL);
				global.LeftToe_P4.transform(World.Current.getRootFrame(),
						transformL);
			} else if (global.mode == Mode_Right) {
				global.RightToe_P3.transform(World.Current.getRootFrame(),
						transformR);
				global.RightToe_P4.transform(World.Current.getRootFrame(),
						transformR);
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
		long localstartTime = 0;
		long localendTime = 0;
		double localcycleTime = 0;
		localstartTime = System.currentTimeMillis();
		boolean bResult = false;
		int cnt = 0;
		mNutCommThread.onSendNutStop();
		// TIGHTENING_OK = false;
		ThreadUtil.milliSleep(NUT_STOP_DELAY);
		JointImpedanceControlMode jicm = new JointImpedanceControlMode(3000,
				3000, 3000, 3000, 3000, 3000, 10);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(
				0.8));
		ThreadUtil.milliSleep(TOE_LOOSE_MOVE_DELAY);
		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(
				0.8).setMode(jicm));
		ThreadUtil.milliSleep(TOE_LOOSE_MOVE_DELAY);
		try {
			switch (nLooseType) {
			case NUT_LOOSE:
				logWrite("Nut Loose", true);
				// nut.selectIO(NUT_LOOSE);
				mNutCommThread.onSendNutProgramSel(NUT_LOOSE);
				int cnts = 0;
				while (PSET_STATE == 2) {
					if (cnts == 99) {
						mNutCommThread.onSendNutProgramSel(NUT_LOOSE);
					}
					if (cnts > 199) {
						cnts = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cnts++;
				}
				break;
			case NUT_ROT_100:
				logWrite("Rot 100", true);
				// nut.selectIO(NUT_ROT_100);
				mNutCommThread.onSendNutProgramSel(NUT_ROT_100);
				int cntss = 0;
				while (PSET_STATE == 8) {
					if (cntss == 99) {
						mNutCommThread.onSendNutProgramSel(NUT_ROT_100);
					}
					if (cntss > 199) {
						cntss = 0;
						break;
					}
					ThreadUtil.milliSleep(10);
					cntss++;
				}
				break;
			}
			ThreadUtil.milliSleep(IO_SELECT_DELAY);

			// nut.RunNutRunnerCW();
			TIGHTENING_STATE = 0;
			mNutCommThread.onSendNutStart();

			// while (!nut.CheckTightenOK()) {
			while (TIGHTENING_STATE != 0) {
				cnt++;
				// if (nut.CheckTightenNOK()) {
				if (TIGHTENING_STATE == 1) {
					logWrite("Tool Loose NOK", true);
					// nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
					return false;
				}

				if (cnt > 500) {
					logWrite("Tool Loose Fail", true);
					// nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					TIGHTENING_STATE = 0;
					return false;
				}
				ThreadUtil.milliSleep(10);
			}

			// nut.StopNutRunner();
			mNutCommThread.onSendNutStop();
			TIGHTENING_STATE = 0;
			logWrite("Tool Loose Complete - " + nLooseType, true);
			bResult = true;
		} catch (Exception ex) {
			getLogger().error(ex.toString());
		}
		localendTime = System.currentTimeMillis();
		localcycleTime = (Math.abs(localstartTime - localendTime) / 1000);

		String strFormat = String.format("%.2f", localcycleTime);

		logWrite("ToeLoosening Cycle Time : " + strFormat + " Sec", false);
		return bResult;
	}

	/*
	 * public boolean fastTightAngle() { int nDagi = 0;
	 * logWrite("Fast Tight angle", true); nut.selectIO(FAST_TIGHT);
	 * ThreadUtil.milliSleep(IO_SELECT_DELAY);
	 * 
	 * CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
	 * ctim.parametrize(CartDOF.X).setStiffness(2000);
	 * ctim.parametrize(CartDOF.Y).setStiffness(2000);
	 * ctim.parametrize(CartDOF.Z).setStiffness(2000);
	 * ctim.parametrize(CartDOF.ROT).setStiffness(300);
	 * tcp.move(lin(lbr.getCurrentCartesianPosition
	 * (tcp)).setJointVelocityRel(1.0).setMode(ctim)); //
	 * tcp.move(lin(lbr.getCurrentCartesianPosition
	 * (tcp)).setJointVelocityRel(1.0)); ThreadUtil.milliSleep(50);
	 * 
	 * tcp.move(linRel(0, 0, 10).setMode(ctim).setJointVelocityRel(0.5));
	 * 
	 * // IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1, //
	 * TimeUnit.MILLISECONDS));
	 * 
	 * nut.RunNutRunnerCW();
	 * 
	 * while (!nut.CheckTightenNOK() && !nut.CheckTightenOK()) { if (nDagi >
	 * 500) { nDagi = 0; return false; } nDagi++; ThreadUtil.milliSleep(10); }
	 * nut.StopNutRunner();
	 * 
	 * nDagi = 0;
	 * 
	 * // nut.StopNutRunner();
	 * 
	 * // hold.cancel(); logWrite("Fast Tight angle Complete", true); return
	 * true; }
	 */

	/*
	 * public boolean toeReLoose() { boolean looseOk = false;
	 * 
	 * // GripperHold(); nut.StopNutRunner();
	 * ThreadUtil.milliSleep(NUT_STOP_DELAY); try {
	 * 
	 * fastTightAngle();
	 * 
	 * ThreadUtil.milliSleep(500);
	 * 
	 * if (!ToeTight(NUT_TIGHT_10N)) { logWrite("10N Tighte fail", true); return
	 * false; }
	 * 
	 * ThreadUtil.milliSleep(500);
	 * 
	 * if (!ToeLoose(NUT_LOOSE)) { logWrite("loose fail", true); return false; }
	 * looseOk = true; } catch (Exception ex) {
	 * getLogger().error(ex.toString()); }
	 * 
	 * return looseOk; }
	 */

	public boolean ToeTight(int nTightType) {
		boolean bTightResult = false;
		int cnt = 0;
		// nut.StopNutRunner();
		mNutCommThread.onSendNutStop();
		ThreadUtil.milliSleep(NUT_STOP_DELAY);

		CartesianImpedanceControlMode ctim = new CartesianImpedanceControlMode();
		ctim.parametrize(CartDOF.X).setStiffness(2000);
		ctim.parametrize(CartDOF.Y).setStiffness(2000);
		ctim.parametrize(CartDOF.Z).setStiffness(2000);
		ctim.parametrize(CartDOF.ROT).setStiffness(300);

		switch (nTightType) {
		case NUT_TIGHT_50N:
			// nut.selectIO(NUT_TIGHT_50N);
			mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
			int cnts = 0;
			while (PSET_STATE == 4) {
				if (cnts == 99) {
					mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
				}
				if (cnts > 199) {
					cnts = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cnts++;
			}
			break;
		// case NUT_TIGHT_10N:
		// nut.selectIO(NUT_TIGHT_10N);
		// break;
		default:
			mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
			int cntss = 0;
			while (PSET_STATE == 4) {
				if (cntss == 99) {
					mNutCommThread.onSendNutProgramSel(NUT_TIGHT_50N);
				}
				if (cntss > 199) {
					cntss = 0;
					break;
				}
				ThreadUtil.milliSleep(10);
				cntss++;
			}
			break;
		}
		ThreadUtil.milliSleep(IO_SELECT_DELAY);

		tcp.move(lin(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(
				0.8));

		// IMotionContainer hold = tcp.moveAsync(positionHold(ctim, -1,
		// TimeUnit.MILLISECONDS));
		tcp.move(positionHold(ctim, 300, TimeUnit.MILLISECONDS));

		// nut.RunNutRunnerCW();
		TIGHTENING_STATE = 0;
		mNutCommThread.onSendNutStart();
		while (true) {
			cnt++;
			if (cnt > 1500) {
				logWrite("Tool Tight TimeOut Fail", true);
				break;
			} else {
				// if (nut.CheckTightenOK()) {
				if (TIGHTENING_STATE == 2) {
					bTightResult = true;
					// nut.StopNutRunner();
					mNutCommThread.onSendNutStop();
					FinalToeTorqueData = ToeTorque;
					TIGHTENING_STATE = 0;
					logWrite("Tool Tight Type : " + nTightType + " - Tight OK",
							true);
					break;
					// } else if (nut.CheckTightenNOK()) {
				} else if (TIGHTENING_STATE == 1) {
					bTightResult = false;
					mNutCommThread.onSendNutStop();
					FinalToeTorqueData = ToeTorque;
					TIGHTENING_STATE = 0;
					logWrite(
							"Tool Tight Type : " + nTightType + " - Tight NOK",
							true);
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
			mNutCommThread.onSendNutStop();

			whileMain = 0;
			// nut.IOClear();
			thread_main = false;
			client_main.endComm();
			if (use_graph)
				ftSender.interrupt();
			t_main.interrupt();
			mNutCommThread.killComm();
			mNut_main.interrupt();
			getLogger().info("Stop Thread!");
			ThreadUtil.milliSleep(1000);
		} catch (Exception e) {
		}
		super.dispose();
	}

	// Runnable class for Packet
	static class MainPRunnable implements Runnable {

		private static final int ARRAYCNT = 20;
		private double[] MOV_LTOE = new double[ARRAYCNT];
		private double[] MOV_RTOE = new double[ARRAYCNT];

		private double AVG_LTOE = 0;
		private double AVG_RTOE = 0;
		private int DATACNT = 0;
		private int BUFCNT = 0;
		private int LHDATACHANGECNT = 0;
		private int RHDATACHANGECNT = 0;

		private final double DATA_CHANGE_VAR = 0.003;
		private final int DATA_CHANGE_CNT = 5;

		private int GLOBALDATACNT = 0;

		public void ToeDataChangeCheck() {
			if (global.workState == 0) {// SIGNAL_RESET
				for (int i = 0; i < 30; i++) {
					if (i < 20) {
						MOV_LTOE[i] = 0;
						MOV_RTOE[i] = 0;
					}
					GLOBAL_MOV_LTOE[i] = 0;
					GLOBAL_MOV_RTOE[i] = 0;
					LHDATACHANGECNT = 0;
					RHDATACHANGECNT = 0;
					IsValueChange_LH_Toe = false;
					IsValueChange_RH_Toe = false;
					DATACNT = 0;
					BUFCNT = 0;
					GLOBALDATACNT = 0;
				}
			}

			MOV_LTOE[BUFCNT] = LToe_Val;
			MOV_RTOE[BUFCNT] = RToe_Val;

			AVG_LTOE = 0;
			AVG_RTOE = 0;
			for (int i = 0; i < DATACNT; i++) {
				AVG_LTOE += MOV_LTOE[i];
				AVG_RTOE += MOV_RTOE[i];
			}
			AVG_LTOE /= DATACNT;
			AVG_RTOE /= DATACNT;

			if (GLOBALDATACNT < 30 - 1) {
				GLOBAL_MOV_LTOE[GLOBALDATACNT] = AVG_LTOE;
				GLOBAL_MOV_RTOE[GLOBALDATACNT] = AVG_RTOE;
				GLOBALDATACNT++;
			} else {
				// 전역버퍼 이동 정렬
				for (int i = 0; i < 30 - 2; i++) {
					GLOBAL_MOV_LTOE[i + 1] = GLOBAL_MOV_LTOE[i];
					GLOBAL_MOV_RTOE[i + 1] = GLOBAL_MOV_RTOE[i];
				}
				GLOBALDATACNT = 0;
			}

			LhToeChangeVal = Math.abs(AVG_LTOE - LToe_Val);
			RhToeChangeVal = Math.abs(AVG_RTOE - RToe_Val);

			if (LhToeChangeVal < DATA_CHANGE_VAR) {
				if (LHDATACHANGECNT < DATA_CHANGE_CNT) {
					LHDATACHANGECNT++;
				} else {
					IsValueChange_LH_Toe = false;
				}
			} else {
				LHDATACHANGECNT = 0;
				IsValueChange_LH_Toe = true;
			}

			if (RhToeChangeVal < DATA_CHANGE_VAR) {
				if (RHDATACHANGECNT < DATA_CHANGE_CNT) {
					RHDATACHANGECNT++;
				} else {
					IsValueChange_RH_Toe = false;
				}
			} else {
				RHDATACHANGECNT = 0;
				IsValueChange_RH_Toe = true;
			}

			if (DATACNT < ARRAYCNT)
				DATACNT++;

			if (BUFCNT < ARRAYCNT - 1)
				BUFCNT++;
			else {
				// 버퍼 이동 정렬
				for (int i = 0; i < ARRAYCNT - 2; i++) {
					MOV_LTOE[i + 1] = MOV_LTOE[i];
					MOV_RTOE[i + 1] = MOV_RTOE[i];
				}
				BUFCNT = 0;
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
			System.arraycopy(global.DoubletoByte8_Array(global.lhToe), 0, send,
					6, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.rhToe), 0, send,
					14, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.lhCamber), 0,
					send, 22, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.rhCamber), 0,
					send, 30, 8);

			System.arraycopy(global.DoubletoByte8_Array(global.visionLeftX), 0,
					send, 38, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionLeftY), 0,
					send, 46, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionRightX),
					0, send, 54, 8);
			System.arraycopy(global.DoubletoByte8_Array(global.visionRightY),
					0, send, 62, 8);

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
					Arr_LToe_Val.add(LToe_Val);
					Arr_RToe_Val.add(RToe_Val);
					Arr_LCamber_Val.add(LCamber_Val);
					Arr_RCamber_Val.add(RCamber_Val);

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
		if (tightState == 1) {
			TIGHTENING_STATE = 2;
			ToeTorque = torque;
			Angle = angle;
			// TIGHTENING_NOK = false;
		}
		if (tightState == 0) {
			TIGHTENING_STATE = 1;
			ToeTorque = torque;
			Angle = angle;
			// TIGHTENING_NOK = true;
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
		PSET_STATE = Pset;
	};
}