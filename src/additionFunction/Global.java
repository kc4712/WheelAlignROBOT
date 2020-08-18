package additionFunction;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.kuka.roboticsAPI.geometricModel.Frame;

public class Global {

	//frame
	public Frame Home;
	public Frame Bypass;
	
	public Frame LeftReady;
	public Frame RightReady;
	
	public Frame LeftTilt;
	public Frame RightTilt;
	
	public Frame LeftVision;
	public Frame RightVision;
	
	public Frame LeftFix_P1;
	public Frame LeftFix_P2;
	public Frame LeftFix_P3;
	public Frame LeftFix_P4;
	public Frame LeftFix_P5;
	public Frame RightFix_P1;
	public Frame RightFix_P2;
	public Frame RightFix_P3;
	public Frame RightFix_P4;
	public Frame RightFix_P5;
	
	public Frame LeftCamber_P1;
	public Frame LeftCamber_P2;
	public Frame LeftCamber_P3;
	public Frame LeftCamber_P4;
	public Frame LeftCamber_P5;
	public Frame LeftCamber_P6;
	public Frame LeftCamber_P7;
	
	public Frame RightCamber_P1;
	public Frame RightCamber_P2;
	public Frame RightCamber_P3;
	public Frame RightCamber_P4;
	public Frame RightCamber_P5;
	public Frame RightCamber_P6;
	public Frame RightCamber_P7;

	public Frame LeftToe_P1;
	public Frame LeftToe_P2;
	public Frame LeftToe_P3;
	public Frame LeftToe_P4;
	public Frame LeftToe_P5;
	public Frame LeftToe_P6;
	public Frame LeftToe_P7;
	
	public Frame RightToe_P1;
	public Frame RightToe_P2;
	public Frame RightToe_P3;
	public Frame RightToe_P4;
	public Frame RightToe_P5;
	public Frame RightToe_P6;
	public Frame RightToe_P7;
	
	
	
	//Server packet
	public byte[] packetHeader;
	public byte carType;
	public byte adjustPart;
	public byte mode;
	public byte workSignal;
	public double lhToe;
	public double rhToe;
	public double lhCamber;
	public double rhCamber;

	public double visionLeftX;
	public double visionLeftY;
	public double visionRightX;
	public double visionRightY;
	
	public byte F_LH_Toe_Count;
	public byte F_RH_Toe_Count;
	public byte[] Barcode;
	
	public byte workState;
	
	public double leftToeTorque;
	public double rightToeTorque;
	public double leftCamberTorque;
	public double rightCamberTorque;
	
	public void Packet(byte[] data) {
		this.packetHeader = new byte[2];
		// parsing
		this.packetHeader[0] = data[0];
		this.packetHeader[1] = data[1];
		if ((this.packetHeader[0] == (byte) 255)
				&& (this.packetHeader[1] == (byte) 170)) {
			this.carType = data[2];
			this.adjustPart = data[3];
			this.mode = data[4];
			this.workSignal = data[5];
			this.lhToe = byte8_ArrayToDouble(data, 6); // 13
			this.rhToe = byte8_ArrayToDouble(data, 14); // 21
			this.lhCamber = byte8_ArrayToDouble(data, 22); // 29
			this.rhCamber = byte8_ArrayToDouble(data, 30); // 37
			this.visionLeftX = byte8_ArrayToDouble(data, 38); // 45
			this.visionLeftY = byte8_ArrayToDouble(data, 46); // 53
			this.visionRightX = byte8_ArrayToDouble(data, 54); // 61
			this.visionRightY = byte8_ArrayToDouble(data, 62); // 69
			this.F_LH_Toe_Count = data[70];
			this.F_RH_Toe_Count = data[71];
		}
	}
	private double byte8_ArrayToDouble(byte[] data, int startIndex) {
		long accum = 0;
		int i = 0;
		for (int shiftBy = 0; shiftBy < 64; shiftBy += 8) {
			accum |= ((long) (data[startIndex + i] & 0xff)) << shiftBy;
			i++;
		}
		return Double.longBitsToDouble(accum);
	}
	
	public byte[] DoubletoByte8_Array(double data)
	{
		byte[] b_a = new byte[8];
		ByteBuffer bb = ByteBuffer.wrap(b_a);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		bb.putDouble(data);
		
		return b_a;
	}
	

}
