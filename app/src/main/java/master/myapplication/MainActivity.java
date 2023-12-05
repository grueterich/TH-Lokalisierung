package master.myapplication;

import static org.pytorch.Device.CPU;

import androidx.appcompat.app.AppCompatActivity;

import android.content.res.AssetFileDescriptor;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;


import org.pytorch.Device;
import org.pytorch.IValue;
import org.pytorch.LiteModuleLoader;
import org.pytorch.Module;
import org.pytorch.PyTorchAndroid;
import org.pytorch.Tensor;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.FloatBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Arrays;
import java.util.Map;

import javax.vecmath.Quat4d;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private float position[] = new float[3];
    private float velocity[] = new float[3];
    private float accelaration[] = new float[3];
    private float lastAcc[][] = new float[200][3];
    private float gravity[]=new float[3];
    private float magno[]=new float[3];
    Interpreter tflite;
   // private Queue<float> lastAcceleration= new Queue() {
   // };
    private float currentRotation;
    private Interpreter tfHelper;
    private SensorManager mSensorManager;
    private Sensor mAccelerometerUncal;
    private Sensor mRotationUncal;
    private Sensor mMangonemter;

    private long lastTimeStampLinear;
    private long lastTimeStampRotation;
    private Module module;

    private Quat4d ori_q=new Quat4d();;
    private Quat4d acc_uncal_q=new Quat4d();;
    private Quat4d gyro_uncal_q=new Quat4d();;
    private Quat4d gyro_q=new Quat4d();
    private Quat4d gyro_q_temp=new Quat4d();
    private Quat4d gyro_q_inv= new Quat4d();

    private Quat4d acc_q= new Quat4d();
    private Quat4d acc_q_temp= new Quat4d();
    private Quat4d acc_q_inv= new Quat4d();
    private float acc[] = new float[3];
    private float gyro[] = new float[3];
    private float acc_scale[] = {1,0.99f,1.02f};
    private float acc_bias[] = {0.05f,0.12f,-0.07f};
    private float gyro_bias[] = {0.03f,0.01f,-0.01f};
    private float gyrofinal[] = new float[3];
    private float accfinal[] = new float[3];
    private float feature[] = new float[72*6];

    private int accRunner=0;
    private int gyroRunner=0;

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
        mAccelerometerUncal = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        mRotationUncal= mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        mMangonemter=mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    private void changePosition(float x, float y, float z) {
        Log.d("Position",x+" "+y+" "+z);
        //this.position[0] = (float) (x*Math.cos(this.currentRotation)+y*Math.sin(this.currentRotation));
        //this.position[1] = (float) (-x*Math.sin(this.currentRotation)+y*Math.cos(this.currentRotation));
        Log.d("Position",this.position[0]+" "+this.position[1]+" "+z);
        //We rotate alonge the z-azis, so this value stays the same
        //this.position[2] = z;
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

     //   changePosition(newPositionX,newPositionY,newPositionZ);
    }
    private void updateRotation(float rotation_z) {
        this.currentRotation=rotation_z;
        //Position stays the same, but with the values change with the new rotation
       // changePosition(this.position[0],this.position[1],this.position[2]);
    }


    private void changeVelocity(float v) {
        this.velocity[0] = v;
        String velOutput = Float.toString(v);
        TextView vlValue = findViewById(R.id.velocity_value);
        vlValue.setText(velOutput);
    }

    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometerUncal, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, mRotationUncal, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, mMangonemter, SensorManager.SENSOR_DELAY_NORMAL);

        Log.d("error", "alles fine");
    }

    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }


    public void onSensorChanged(SensorEvent event) {
       // useNN();
        Log.d("Sensorevent","Event: "+event.sensor.getName()+" "+event.sensor.getType()+" "+event.timestamp);
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) { //Sensor.TYPE_ACCELEROMETER Sensor.Type //Sensor.TYPE_ACCELEROMETER_UNCALIBRATED /7Sensor.TYPE_LINEAR_ACCELERATION
            updatePosition(event.values[0],event.values[1],event.values[2],event.timestamp);
            ((TextView) findViewById(R.id.accValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.accValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.accValueZ)).setText(Float.valueOf(event.values[2]).toString());
            acc[0]=acc_scale[0]*(event.values[0]-acc_bias[0]);
            acc[1]=acc_scale[1]*(event.values[1]-acc_bias[1]);
            acc[2]=acc_scale[2]*(event.values[2]-acc_bias[2]);

            acc_uncal_q=new Quat4d(acc[0],acc[1],acc[2],0);
            acc_q_inv.inverse(acc_uncal_q);
            acc_q_temp.mul(acc_uncal_q,ori_q);
            acc_q.mul(acc_q_temp,gyro_q_inv);
            accfinal[0]=(float)acc_q.x;
            accfinal[1]=(float)acc_q.y;
            accfinal[2]=(float)acc_q.z;
            if(accRunner<72) {
                Log.d("Acc", String.valueOf(accfinal[0]));
                System.arraycopy(accfinal, 0, feature, 3 + 6 * accRunner, 3);
                Log.d("Acc2", Arrays.toString(feature));
                this.position[2]=feature[3 + 6 * accRunner];
                changePosition(this.position[0],this.position[1],this.position[2]);
                changeVelocity(accRunner);
                accRunner++;
            }else if (gyroRunner==72 && accRunner==72){
                useNN();
                accRunner=0;
                gyroRunner=0;
            }
          //  accRunner++;
        }
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) { //Sensor.TYPE_GYROSCOPE_UNCALIBRATED
            updateRotation(event.values[2]);
            ((TextView) findViewById(R.id.rotateValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.rotateValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.rotateValueZ)).setText(Float.valueOf(event.values[2]).toString());

            gyro[0]=event.values[0]-gyro_bias[0];
            gyro[1]=event.values[1]-gyro_bias[1];
            gyro[2]=event.values[2]-gyro_bias[2];

            gyro_uncal_q= new Quat4d(gyro[0],gyro[1],gyro[2],0);
            gyro_q_inv.inverse(gyro_uncal_q);
            gyro_q_temp.mul(gyro_uncal_q,ori_q);
            gyro_q.mul(gyro_q_temp,gyro_q_inv);
            gyrofinal[0]=(float)gyro_q.x;
            gyrofinal[1]=(float)gyro_q.y;
            gyrofinal[2]=(float)gyro_q.z;
            if(gyroRunner<72) {
                System.arraycopy(gyrofinal, 0, feature, 0 + 6 * gyroRunner, 3);
                gyroRunner++;
            }else if (gyroRunner==72 && accRunner==72){
                useNN();
                accRunner=0;
                gyroRunner=0;
            }
               // gyroRunner++;

        }
        /*if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) { //Sensor.TYPE_GYROSCOPE_UNCALIBRATED
            updateRotation(event.values[2]);
            ((TextView) findViewById(R.id.rotateValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.rotateValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.rotateValueZ)).setText(Float.valueOf(event.values[2]).toString());
        }
        */
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            magno= event.values;
        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            gravity= event.values;
        }
        if (gravity != null && magno != null) {
            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, gravity, magno);
            if (success) {

                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                //Log.d("Sensorevent","Event: "+"Orientation0"+" "+orientation[2]);
                //Log.d("Sensorevent","Event: "+"Orientation0"+" "+orientation[0]+" Orientation1"+" "+orientation[1]+" Orientation2"+" "+orientation[2]);
                //Log.d("Sensorevent2","Event: "+"R"+" "+R[0]+" I"+" "+I[0]);
              //  for (int xp=0;xp<R.length;xp++){
              //      Log.d("Sensorevent2","Event: "+"R "+xp+" "+R[xp]);
               // }

                ori_q= toQuaternion(orientation[2],orientation[1],orientation[0]);
             //   ori_q = new Quat4d(orientation[1]/(Math.PI),orientation[2]/(Math.PI/2),orientation[0]/(Math.PI),0);
            }
        }
    }

    Quat4d toQuaternion(double rollDeg, double pitchDeg, double azimuthDeg) // We get the orientation in degree
    {
        double roll=rollDeg*(Math.PI/180);
        double pitch=pitchDeg*(Math.PI/180);
        double azimuth=azimuthDeg*(Math.PI/180);

        // Abbreviations for the various angular functions
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin((pitch * 0.5));
        double cy = Math.cos(azimuth * 0.5);
        double sy = Math.sin((azimuth * 0.5));

        Quat4d q=new Quat4d();
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }
    private void moveUpAcc(float value, float value1, float value2) {
        for(int i=0;i<199;i++){
            lastAcc[i][0]=lastAcc[i+1][0];
            lastAcc[i][1]=lastAcc[i+1][1];
            lastAcc[i][2]=lastAcc[i+1][2];
            
        }
        lastAcc[199][0]=value;
        lastAcc[199][1]=value1;
        lastAcc[199][2]=value2;

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void useNN(){

// Load in the model
        try {
            //String file=assetFilePath("checkpoint_jit_latest.ptl");
            //String file=assetFilePath("checkpoint_jit_latest_light.ptl");
            //String file=assetFilePath("model.onnx");


            String file=assetFilePath("checkpoint_jit_latest_light.ptl");
            //String file=assetFilePath("ronin_lstm_checkpoint");
            module = LiteModuleLoader.load(file,null, CPU);
            Log.d("model2",module.toString());
        } catch (Exception e) {
            Log.e("model", "Unable to load model for file", e);
        }

         feature= new float[2*6];
     /*    for(int i=0;i<72;i++) {
            gyrofinal[0]=i;
            gyrofinal[1]=i;
            gyrofinal[2]=+i;
            accfinal[0]=-i;
            accfinal[1]=-i;
            accfinal[2]=-i;
            System.arraycopy(gyrofinal, 0, feature, 0+6*i, 3);
            System.arraycopy(accfinal, 0, feature, 3+6*i, 3);
        }
*/
        long[] shape={72,1,6};


        Tensor inTensor = Tensor.fromBlob(feature,shape);
        Log.d("Tensor",inTensor.toString());
        Log.d("Tensorwerte", Arrays.toString(inTensor.getDataAsFloatArray()));
        IValue value=IValue.from(inTensor);
        try {

             Tensor res = module.forward(value).toTensor();
            // Log.d("Ergebnis", res.toString());
             float[] x_and_y=res.getDataAsFloatArray();
            this.position[0]=x_and_y[0];
            this.position[1]=x_and_y[1];
             Log.d("Ergebnise", Arrays.toString(x_and_y));

           /* long[] shape2= {2, 1, 6};
            feature= new float[2*6];
            for(int i=0;i<2;i++) {
                gyrofinal[0]=i;
                gyrofinal[1]=i;
                gyrofinal[2]=+i;
                accfinal[0]=-i;
                accfinal[1]=-i;
                accfinal[2]=-i;
                System.arraycopy(gyrofinal, 0, feature, 0+6*i, 3);
                System.arraycopy(accfinal, 0, feature, 3+6*i, 3);
            }
            Tensor inTensor2 = Tensor.fromBlob(feature,shape2);
            Log.d("Tensor",inTensor2.toString());
            Log.d("Tensorwerte", Arrays.toString(inTensor2.getDataAsFloatArray()));
            value=IValue.from(inTensor2);
            res = module.forward(value).toTensor();
            x_and_y=res.getDataAsFloatArray();
            Log.d("Ergebnise", Arrays.toString(x_and_y));
*/
        }catch(Error e){
            Log.e("Ergebnis",e.toString());
        }

/*
      try {
          MappedByteBuffer buffer=loadFile();
          tflite = new Interpreter(buffer);
          Log.d("Model",tflite.toString());
        }catch (Exception ex){
            Log.d("Model","Model not found", ex);
            ex.printStackTrace();
        }
      tflite.toString();
        float inputVal[] = new float[6];
        System.arraycopy(gyrofinal,0,inputVal,0,3);
        System.arraycopy(accfinal,0,inputVal,3,3);
        float output[]=new float[2];
        tflite.run(inputVal,output);

*/
        changeVelocity(accRunner);

    }
    public String assetFilePath(String assetName) throws IOException {
        File file = new File(this.getFilesDir(), assetName);
        if (file.exists() && file.length() > 0) {
            Log.d("model",file.getAbsolutePath());
            return file.getAbsolutePath();
        }

        try (InputStream is = this.getAssets().open(assetName)) {
            try (OutputStream os = new FileOutputStream(file)) {
                byte[] buffer = new byte[4 * 1024];
                int read;
                while ((read = is.read(buffer)) != -1) {
                    os.write(buffer, 0, read);
                }
                os.flush();
            }
            Log.d("model",file.getAbsolutePath());
            return file.getAbsolutePath();
        }
    }


    private MappedByteBuffer loadFile() throws IOException {
        AssetFileDescriptor fileDescriptor=this.getAssets().openFd("checkpoint_latest.tflite");
        FileInputStream inputStream=new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel=inputStream.getChannel();
        long startOffset=fileDescriptor.getStartOffset();
        long declareLength=fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY,startOffset,declareLength);
    }



}


