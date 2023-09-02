package master.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import org.tensorflow.lite.Interpreter;

import java.io.File;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private float position[] = new float[3];
    private float velocity[] = new float[3];
    private float accelaration[] = new float[3];
    private float currentRotation;
    private Interpreter tfHelper;
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;
    private Sensor mRotationalVelocity;
    private long lastTimeStampLinear;
    private long lastTimeStampRotation;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.position[0] = 0;
        this.position[1] = 0;
        this.position[2] = 0;
        this.velocity[0] = 0;
        this.velocity[1] = 0;
        this.velocity[2] = 0;
        this.currentRotation=0;
        setContentView(R.layout.activity_main);

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mRotationalVelocity = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    }

    private void changePosition(float x, float y, float z) {
        Log.d("Position",x+" "+y+" "+z);
        this.position[0] = (float) (x*Math.cos(this.currentRotation)+y*Math.sin(this.currentRotation));
        this.position[1] = (float) (-x*Math.sin(this.currentRotation)+y*Math.cos(this.currentRotation));
        Log.d("Position",this.position[0]+" "+this.position[1]+" "+z);
        //We rotate alonge the z-azis, so this value stays the same
        this.position[2] = z;
        String xOutput = Float.toString(x);
        TextView xValue = findViewById(R.id.x_value);
        xValue.setText(xOutput);
        String yOutput = Float.toString(y);
        TextView yValue = findViewById(R.id.y_value);
        yValue.setText(yOutput);
        String zOutput = Float.toString(z);
        TextView zValue = findViewById(R.id.z_value);
        zValue.setText(zOutput);
    }
    private void updatePosition(float accileration_x, float accileration_y, float accileration_z, long timeStamp) {
        if(this.lastTimeStampLinear == 0) {
            this.lastTimeStampLinear = timeStamp;
        }
        double timeInterval = (timeStamp - lastTimeStampLinear) / 1000000000.0;
        this.accelaration[0]=accileration_x;
        this.accelaration[1]=accileration_y;
        this.accelaration[2]=accileration_z;

        this.velocity[0]=(float)(this.accelaration[0]*timeInterval+this.velocity[0]);
        this.velocity[1]=(float)(this.accelaration[1]*timeInterval+this.velocity[1]);
        this.velocity[2]=(float)(this.accelaration[2]*timeInterval+this.velocity[2]);

        float newPositionX=(float)(this.position[0]+0.5*this.accelaration[0]*timeInterval*timeInterval+this.velocity[0]*timeInterval);
        float newPositionY=(float)(this.position[1]+0.5*this.accelaration[1]*timeInterval*timeInterval+this.velocity[1]*timeInterval);
        float newPositionZ=(float)(this.position[2]+0.5*this.accelaration[2]*timeInterval*timeInterval+this.velocity[2]*timeInterval);
        Log.d("Position",this.velocity[0]+" "+this.velocity[1]+" "+this.velocity[2]);

        changePosition(newPositionX,newPositionY,newPositionZ);
    }
    private void updateRotation(float rotation_z) {
        this.currentRotation=rotation_z;
        //Position stays the same, but with the values change with the new rotation
        changePosition(this.position[0],this.position[1],this.position[2]);
    }


    private void changeVelocity(float v) {
        this.velocity[0] = v;
        String velOutput = Float.toString(v);
        TextView vlValue = findViewById(R.id.velocity_value);
        vlValue.setText(velOutput);
    }

    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, mRotationalVelocity, SensorManager.SENSOR_DELAY_NORMAL);
        Log.d("error", "ales fine");
    }
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }


    public void onSensorChanged(SensorEvent event) {
        Log.d("Sensorevent","Event: "+event.sensor.getName()+" "+event.sensor.getType());
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) { //Sensor.TYPE_ACCELEROMETER Sensor.Type //Sensor.TYPE_ACCELEROMETER_UNCALIBRATED /7Sensor.TYPE_LINEAR_ACCELERATION
            updatePosition(event.values[0],event.values[1],event.values[2],event.timestamp);
            ((TextView) findViewById(R.id.accValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.accValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.accValueZ)).setText(Float.valueOf(event.values[2]).toString());

        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) { //Sensor.TYPE_GYROSCOPE_UNCALIBRATED
            updateRotation(event.values[2]);
            ((TextView) findViewById(R.id.rotateValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.rotateValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.rotateValueZ)).setText(Float.valueOf(event.values[2]).toString());
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
/*
    public void useNN(){
        this.tfHelper.run();
        this.tfHelper.
    }

 */
}


