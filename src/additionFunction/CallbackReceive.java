package additionFunction;

public interface CallbackReceive {
	public void cbToolResult(float torque, int tightState, float angle);
	public void cbToolDinStateResult(int DinState);
	public void cbToolPSetResult(int Pset);
}
