package com.magnin.imuaccess;

import java.util.concurrent.Executor;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
//import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.ToggleButton;

public class MainActivity extends Activity {

	private final int FREE_PORT_MIN = 1000;
	private final int FREE_PORT_MAX = 6535;
	private TCPClient mTCPClient = null;
	private EditText IP = null;
	private EditText port = null;

    private mSensorListener accListener;
    private mSensorListener gyroListener;
    private mSensorListener magListener;
    
	public TextView a = null;
	public TextView g = null;
	public TextView m = null;
    
	@Override
	protected void onCreate (Bundle savedInstanceState) {
		super.onCreate(savedInstanceState); // default
		setContentView(R.layout.activity_main); // default
		
		IP = (EditText) findViewById(R.id.editText1);
		port = (EditText) findViewById(R.id.editText2);
		a = (TextView) findViewById(R.id.textView6);
	   	g = (TextView) findViewById(R.id.TextView01);
	   	m = (TextView) findViewById(R.id.textView7);
		accListener = new mSensorListener(Sensor.TYPE_ACCELEROMETER);
	    gyroListener = new mSensorListener(Sensor.TYPE_GYROSCOPE);
	    magListener = new mSensorListener(Sensor.TYPE_MAGNETIC_FIELD);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) { // default
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}
	
	//@Override
	//protected void onPause() {
		//ToggleButton toggle = (ToggleButton)findViewById(R.id.toggleButton1);
		//super.onPause();
		//if ( ((CheckBox) findViewById(R.id.checkBox1)).isChecked() == false ){
			//if ( toggle.isChecked() ) { // is the toggle on? if yes stop sensors
				//toggle.setChecked(false); // make button unchecked
			//}
		//}
    //}
	
	//@Override
	//protected void onStop() {}

	/* Called when the user toggles the button */
	public void onToggleClicked (View view) {
		
		if ( ((ToggleButton)view).isChecked()) { // is the toggle on?
			
			if ( IP.getText().length() == 0 ||
					port.getText().length() == 0 ||
					Integer.parseInt(port.getText().toString()) < FREE_PORT_MIN ||
					Integer.parseInt(port.getText().toString()) > FREE_PORT_MAX  ) { 
				
	            AlertDialog.Builder alertbox = new AlertDialog.Builder(this);
	            alertbox.setMessage("Invalid IP address or port.");
	            alertbox.setNeutralButton("Ok", new DialogInterface.OnClickListener() {
	                public void onClick(DialogInterface arg0, int arg1) {}
	            });
	            alertbox.show();
	            
	            ((ToggleButton)view).setChecked(false); // make button unchecked
			
			} else {
				view.requestFocus();
				IP.setFocusable(false);
				IP.setTextColor(getResources().getColor(R.color.gray_not_focusable) );
				port.setFocusable(false);
				port.setTextColor(getResources().getColor(R.color.gray_not_focusable) );
				InputMethodManager imm = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
				imm.hideSoftInputFromWindow(IP.getWindowToken(), 0);
				imm.hideSoftInputFromWindow(port.getWindowToken(), 0);
				
				accListener.mSensorManager.registerListener(accListener, accListener.mSensor, SensorManager.SENSOR_DELAY_UI);
				gyroListener.mSensorManager.registerListener(gyroListener, gyroListener.mSensor, SensorManager.SENSOR_DELAY_UI);
				magListener.mSensorManager.registerListener(magListener, magListener.mSensor, SensorManager.SENSOR_DELAY_UI);
				
				new EstablishTCPConnection().execute(mTCPClient);
			}
		
		} else {
			if ( mTCPClient != null ) {
				mTCPClient.stop();
			}
			
			accListener.mSensorManager.unregisterListener(accListener);
			gyroListener.mSensorManager.unregisterListener(gyroListener);
			magListener.mSensorManager.unregisterListener(magListener);
			
			IP.setFocusable(true);
			IP.setFocusableInTouchMode(true);
			IP.setTextColor(getResources().getColor(R.color.black) );
			port.setFocusable(true);
			port.setFocusableInTouchMode(true);
			port.setTextColor(getResources().getColor(R.color.black));
		}
	}

	public class EstablishTCPConnection implements Executor {
		public EstablishTCPConnection() {
			mTCPClient = new TCPClient(
				IP.getText().toString(),
				Integer.parseInt(port.getText().toString()),
				new TCPClient.OnMessageReceived() {
					@Override
					public void messageReceived(String message) {
						
						if ( message.equals("0") ) { // acceleration
							mTCPClient.msgSender.sendMessage( Float.toString(accListener.values[0]) + "," + Float.toString(accListener.values[1]) + "," + Float.toString(accListener.values[2]) );
						} else if ( message.equals("1") ) { // Angular speed
							//g.setText(Float.toString(gyroListener.values[0]) + ", " + Float.toString(gyroListener.values[1]) + ", " + Float.toString(gyroListener.values[2]));
							mTCPClient.msgSender.sendMessage( Float.toString(gyroListener.values[0]) + "," + Float.toString(gyroListener.values[1]) + "," + Float.toString(gyroListener.values[2]) );
						} else if ( message.equals("2") ) { // Magnetic field
							mTCPClient.msgSender.sendMessage( Float.toString(magListener.values[0]) + "," + Float.toString(magListener.values[1]) + "," + Float.toString(magListener.values[2]) );
						} else if ( message.equals("3") ) { // the previous three
							mTCPClient.msgSender.sendMessage( Float.toString(accListener.values[0]) + "," + Float.toString(accListener.values[1]) + "," + Float.toString(accListener.values[2]) + "," +
									Float.toString(gyroListener.values[0]) + "," + Float.toString(gyroListener.values[1]) + "," + Float.toString(gyroListener.values[2]) + "," +
									Float.toString(magListener.values[0]) + "," + Float.toString(magListener.values[1]) + "," + Float.toString(magListener.values[2]) + ",\0");
						} else {
							System.err.println("Server msg doesn't match any option.");
							mTCPClient.msgSender.sendMessage("Cannot answer your request");
						}
					}
				},
				new TCPClient.OnSendingMessage() {
					@Override
					public void sendMessage (String message) {
						if (mTCPClient.out != null && !mTCPClient.out.checkError() && message != null) {
							//System.out.println(message);
							mTCPClient.out.write(message);
							mTCPClient.out.flush();
						}
					}
				}); // end new
		}
			
		public void execute(Runnable r) {
			new Thread(r).start();
		}
	}
	
	public class mSensorListener implements SensorEventListener {
	     private final SensorManager mSensorManager;
	     private final Sensor mSensor;
	     public float[] values = new float[3];
	     final int type;

	     public mSensorListener( int type ) {
	         mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
	         mSensor = mSensorManager.getDefaultSensor(type);
	         this.type = type;
	     }

	     public void onAccuracyChanged(Sensor sensor, int accuracy) {}
	     
	     
	     public void onSensorChanged(SensorEvent event) {
	    	 values[0] = event.values[0];
	    	 values[1] = event.values[1];
	    	 values[2] = event.values[2];
	    	 if (type == Sensor.TYPE_ACCELEROMETER)
	    		 a.setText(String.format("%9.3f", values[0]) + ", " + String.format("%8.3f", values[1]) + ", " + String.format("%8.3f", values[2]));
	    	 else if (type == Sensor.TYPE_GYROSCOPE)
	    		 g.setText(String.format("%9.3f", values[0]) + ", " + String.format("%8.3f", values[1]) + ", " + String.format("%8.3f", values[2]));
	    	 else
	    		 m.setText(String.format("%9.3f", values[0]) + ", " + String.format("%8.3f", values[1]) + ", " + String.format("%8.3f", values[2]));
	     }
	}
}