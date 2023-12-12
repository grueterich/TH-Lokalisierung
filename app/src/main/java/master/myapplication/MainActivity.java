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
    private Sensor mAccelerometer;

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
    private float gyrofinal[] = new float[3];
    private float accfinal[] = new float[3];
    private float feature[] = new float[72*6];

    private float preGyroX[]= {-0.05511540116232253f,
            -0.05641679684788734f,
            -0.061611748290821645f,
            -0.06707177965940125f,
            -0.07001248761481774f,
            -0.07049051729989572f,
            -0.07260669246730903f,
            -0.0743544462888934f,
            -0.07569335380873016f,
            -0.07578392420124054f,
            -0.0737827471167346f,
            -0.07161623022899803f,
            -0.0714284171741275f,
            -0.07337534642011573f,
            -0.07149017807988849f,
            -0.06704607865436937f,
            -0.06134393973690665f,
            -0.0615321704060113f,
            -0.06527813224513235f,
            -0.06102739854244528f,
            -0.05791570574007116f,
            -0.05616924313881399f,
            -0.05175868832171953f,
            -0.04917341307480408f,
            -0.045745647299001005f,
            -0.04259482494487789f,
            -0.036385243172854964f,
            -0.028162146240081475f,
            -0.017647098972350256f,
            -0.004236471672831812f,
            0.007630648487486717f,
            0.022655497286470445f,
            0.03815125141889897f,
            0.05182748799559994f,
            0.0614665490852367f,
            0.06709747029787053f,
            0.06950323079243971f,
            0.06502107506743811f,
            0.058802632949040284f,
            0.05099512248812988f,
            0.04373882706152193f,
            0.03284001482775055f,
            0.01814740797987596f,
            0.003411799278604254f,
            -0.008952817143726203f,
            -0.018867010004547765f,
            -0.026887387711378337f,
            0.033570990349313254f,
            -0.0391846958972361f,
            -0.04163294805868082f,
            -0.043064980255901976f,
            -0.04839124782253444f,
            -0.05169793850447078f,
            -0.05424345099907352f,
            -0.053394552164190916f,
            -0.05545524539146436f,
            -0.057569362380837895f,
            -0.05868983949773801f,
            -0.059260233060378996f,
            -0.058272600178513836f,
            -0.05470059519617321f,
            -0.051634558431295906f,
             -0.04583814276139572f,
             0.03675863703753252f,
             -0.032287587130918206f,
             -0.022854507899331085f,
             -0.019288961575678536f,
              -0.012579754868595285f,
              -0.003360264319560315f,
               -0.002846781701721444f,
            0.0062315037164127635f,
            0.011964042722701481f};
    private float preGyroY[]= {
            0.0027027149347485617f,
            0.005253808225735695f,
            0.00408183302527756f,
            0.001000738443708222f,
            0.0015411168923411191f,
            0.003300050079287348f,
            0.0058702976900201105f,
            0.008400424399566704f,
            0.011628611753679827f,
            0.0156367150457483f,
            0.02028473913287142f,
            0.021572240971988316f,
            0.01989721160003133f,
            0.018231359965002046f,
            0.014859159052496497f,
            0.012499328251395797f,
            0.00830925241753804f,
            0.0059256128927600085f,
            0.00513167203128424f,
            0.0067957901581196445f,
            0.011518844539034383f,
            0.010851158081336553f,
            0.014363146040318419f,
            0.021070848594349784f,
            0.025917956438265104f,
            0.033375731456586136f,
            0.04008841603104683f,
            0.04269506362745027f,
            0.041406488613263154f,
            0.04059698344657094f,
            0.03584646991591024f,
            0.02677888192673138f,
            0.016653268880568832f,
            0.003093274721129467f,
            -0.00643726270181906f,
            -0.006077203429667219f,
            -0.005819406807706575f,
            -0.007134203903255215f,
            -0.004597753826899165f,
            -4.489533406721461E-4f,
            0.0066631313159156574f,
            0.01564436997756699f,
            0.021774963151760907f,
            0.02925054580164888f,
            0.03472597716031578f,
            0.0352719742026209f,
            0.030618062133342214f,
            0.028872760298880664f,
            0.023796811407221122f,
            0.01681262009307804f,
            0.012773433991740264f,
            0.010475593005776588f,
            0.006816033812316562f,
            0.002584100584966265f,
            6.831396363371877E-4f,
            -0.00469774255941572f,
            -0.006113352842636069f,
            -0.004030064714141363f,
            -0.006747531505146117f,
            -0.009207586355498358f,
            -0.010718581000166217f,
            -0.010585235444559778f,
            -0.013801996567016386f,
            -0.017141685796028802f,
            -0.01696382517587027f,
            -0.015664199275586543f,
            -0.017396540930843345f,
            -0.01918479456764112f,
            -0.01622714957795673f,
            -0.01572065455998615f,
            -0.011830967784354607f,
            -0.007261367750728219f,
    };
    private float preGyroZ[]= {0.011170483018711904f,
            0.007143909919588999f,
            0.006207333579777951f,
            0.008949668624643992f,
            0.011179240744163673f,
            0.015252123330680349f,
            0.019086271576464687f,
            0.021938456854517452f,
            0.024696828928608812f,
            0.02782607391912655f,
            0.028610430349888617f,
            0.029526537387679894f,
            0.029434038290113738f,
            0.030039215431334528f,
            0.03150726785617758f,
            0.03346737017491963f,
            0.03848489877229709f,
            0.04395349878042562f,
            0.04696760326679061f,
            0.047487007490049464f,
            0.046375228599494764f,
            0.043894756244574104f,
            0.036907148416813564f,
            0.026070261109095428f,
            0.015391460366522749f,
            0.0065475018769095795f,
            -0.0014991810393950623f,
            -0.008907708272019182f,
            -0.012694835274254739f,
            -0.012994286963841672f,
            -0.011102045009614517f,
            -0.005331954627785268f,
            1.3962753904961118E-4f,
            0.009349923349915527f,
            0.018719661566767035f,
            0.025338681586762852f,
            0.0368719769849536f,
            0.04987214827806402f,
            0.06217420036515274f,
            0.07243240335496554f,
            0.07990006409532718f,
            0.08553653837471924f,
            0.08889581483143721f,
            0.0889922456054213f,
            0.08739227543406394f,
            0.08181626637133659f,
            0.07504345212424691f,
            0.0662629115382752f,
            0.05593307590979366f,
            0.04479862453039886f,
            0.03864533836733349f,
            0.03164818740284888f,
            0.023104029688636767f,
            0.017362955348177642f,
            0.015066907225707037f,
            0.009858494945831953f,
            0.010769432635053849f,
            0.010763612608201675f,
            0.011944026662377287f,
            0.013906806554480068f,
            0.01201375085358272f,
            0.01176171354245227f,
            0.00972824029587456f,
            0.005062267961486192f,
            0.005512562165839661f,
            3.81533579416826E-4f,
            -0.0020398827849911327f,
            -0.005424948679424458f,
            -0.007026800750232655f,
            -0.004974168941109048f,
            -0.004581684743265794f,
            6.426220126942769E-4f
    };
    private float preAccX[]= {
            -9.733019749729559f,
            -9.727884986565256f,
            -9.784895472858807f,
            -9.761783628032747f,
            -9.789389986562936f,
            -9.78001409395139f,
            -9.765921973510913f,
            -9.77649833476337f,
            -9.760222647965655f,
            -9.75792047739984f,
            -9.786188985377292f,
            -9.768664315945465f,
            -9.798371491333643f,
            -9.816746580893883f,
            -9.838998520393613f,
            -9.825256065987068f,
            -9.889614916399296f,
            -9.872212530040558f,
            -9.878376423843422f,
            -9.896438111603231f,
            -9.875577811054434f,
            -9.8400075736818f,
            -9.860917560846636f,
            -9.87912514553988f,
            -9.908484198822215f,
            -9.895884994421266f,
            -9.904068438582163f,
            -9.926096886173138f,
            -9.887250797756822f,
            -9.901838611259052f,
            -9.905889262609792f,
            -9.903237666366962f,
            -9.878557904925874f,
            -9.85138598777654f,
            -9.832570557275076f,
            -9.841447800431707f,
            -9.85800312111629f,
            -9.836873951148151f,
            -9.815609151910882f,
            -9.841001168311164f,
            -9.830270736972738f,
            -9.833602209188435f,
            -9.851531297303191f,
            -9.847051675330803f,
            -9.830601994202008f,
            -9.817144963269348f,
            -9.839227941141464f,
            -9.838195442504272f,
            -9.848230188484271f,
            -9.89758590157146f,
            -9.950463645677218f,
            -9.963016608151548f,
            -9.998091700285823f,
            -10.039623415472152f,
            -10.057853489466815f,
            -10.074535226031857f,
            -10.116107798469038f,
            -10.12613221508225f,
            -10.13509216950091f,
            -10.112721587872276f,
            -10.129431721284025f,
            -10.095904773336894f,
            -10.068984930452963f,
            -10.049348551936783f,
            -9.995844194139403f,
            -9.99799607920608f,
            -9.944968451605634f,
            -9.938571949966024f,
            -9.903602743847852f,
            -9.882921803907983f,
            -9.862412574343406f,
            -9.854863733914696f
    };
    private float preAccY[]= {
            0.02915627865328006f,
            0.050790722052891216f,
            0.0328829349192699f,
            0.024653385432119052f,
            0.04215869888642668f,
            0.013213031993398832f,
            0.020065671868383896f,
            0.025423483518510825f,
            0.02352118621369637f,
            0.05284086594151437f,
            0.04671004670177502f,
            0.05432994831493761f,
            0.019287310805837267f,
            -0.0049842579475988654f,
            0.051481577381748855f,
            0.019825947723325816f,
            0.0618528080420888f,
            0.060059295561547876f,
            0.05875719214890514f,
            0.02328770846018323f,
            0.012950728022014885f,
            0.00819032377250572f,
            -0.009469925366599984f,
            -0.00834658185872407f,
            0.018789323924629016f,
            -0.014122846570698885f,
            0.0019143730884969758f,
            -0.0018539497677591762f,
            -0.011893603760134134f,
            0.005296044813741853f,
            0.029034297380515384f,
            0.02439673583515501f,
            0.08966858770123803f,
            0.1863845685959422f,
            0.2051527538743501f,
            0.21612411607957924f,
            0.17812860861333124f,
            0.15288098618184343f,
            0.15744341274705043f,
            0.1698522072026613f,
            0.13655344522607377f,
            0.10095797626565575f,
            0.05468470015168596f,
            0.0019048195094082052f,
            -0.015230521085215172f,
           -0.014566918151376194f,
            -0.053353550233931346f,
            -0.0461457977140304f,
            -0.045204528710255265f,
            -0.06108247257844142f,
            -0.07154250123745219f,
            -0.07006983369361074f,
            -0.05996966821518888f,
            -0.07076472367922104f,
            -0.03771986897310478f,
            -0.012372683945266512f,
            0.013806807122008201f,
            0.013137338689042759f,
            0.06522700793558116f,
            0.03871294655516932f,
            0.0728200120553616f,
            0.05842803056624393f,
            0.0697436102474218f,
            0.07919160267624144f,
            0.05737974817432537f,
            0.06713268408023275f,
            0.05856054955959814f,
            0.07809939842810446f,
            0.11532195326540351f,
            0.09025871155941244f,
            0.10711923125979506f,
            0.13277766283455017f}
            ;
    private float preAccZ[]= {
            0.566486329678951f,
            0.552058852963638f,
            0.5554099106133354f,
            0.5400259366028164f,
            0.5137769858217779f,
            0.511157812951832f,
            0.4978401448578838f,
            0.4803375533171682f,
            0.4811934891817065f,
            0.46379160455365553f,
            0.4334492745279008f,
            0.430380930523449f,
            0.4482056551118981f,
            0.475219824303534f,
            0.4857766574503151f,
            0.4954405191914963f,
            0.49377039584074106f,
            0.51980591930156f,
            0.49280879327874316f,
            0.5110016289698044f,
            0.48874940729929467f,
            0.46454095068648144f,
            0.42766137775831525f,
            0.3606104259336916f,
            0.34916763184938693f,
            0.37544677918114483f,
            0.38422331099246637f,
            0.4165298500861263f,
            0.4621175061918219f,
            0.5011714433059021f,
            0.5310658871311419f,
            0.5047275428188224f,
            0.5305108558989982f,
            0.5171786682511329f,
            0.49118416881707605f,
            0.47246570206882044f,
            0.489498968648695f,
            0.5230286762867309f,
            0.5571785088627987f,
            0.5910957043801132f,
            0.5898619110015028f,
            0.6097641712002235f,
            0.6553602901018232f,
            0.6981259409016563f,
            0.7307886025108569f,
            0.7862392557939482f,
            0.79000262419893f,
            0.7771639669320992f,
            0.7668898173459294f,
            0.7683429749140619f,
            0.7483704436019325f,
            0.7022537837576504f,
            0.6465950024218861f,
            0.5844299616000687f,
            0.5380973701451086f,
            0.4999742778676568f,
            0.4502097178202368f,
            0.4480949327723784f,
            0.4081884604650079f,
            0.37595950964287667f,
            0.40932028972153894f,
            0.40783572478308394f,
            0.43682108910605627f,
            0.4182107333194069f,
            0.42471492318394194f,
            0.41726149000307666f,
            0.46260160254075916f,
            0.44775064956257515f,
            0.4703363839304042f,
            0.42841830128178404f,
            0.4415977175501892f,
            0.4408058834072747f
    };
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
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
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
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);

        Log.d("error", "alles fine");
    }

    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }


    public void onSensorChanged(SensorEvent event) {

        Log.d("Sensorevent","Event: "+event.sensor.getName()+" "+event.sensor.getType()+" "+event.timestamp);
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) { //Sensor.TYPE_ACCELEROMETER Sensor.Type //Sensor.TYPE_ACCELEROMETER_UNCALIBRATED /7Sensor.TYPE_LINEAR_ACCELERATION
            ((TextView) findViewById(R.id.accValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.accValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.accValueZ)).setText(Float.valueOf(event.values[2]).toString());
         Log.d("Acc", accRunner+"runner!!");


            if(accRunner<72) {
                acc[0]=acc_scale[0]*(event.values[0]-event.values[3]);
                acc[1]=acc_scale[1]*(event.values[1]-event.values[4]);
                acc[2]=acc_scale[2]*(event.values[2]-event.values[5]);
   /////////////////////////////////////////////////////
        /*        acc[0]=preAccX[accRunner];
                acc[1]=preAccY[accRunner];
                acc[2]=preAccZ[accRunner]; */
   ///////////////////////////////////////////////////////////////////
                acc_uncal_q=new Quat4d(acc[0],acc[1],acc[2],0);
                acc_q_inv.inverse(acc_uncal_q);
                acc_q_temp.mul(acc_uncal_q,ori_q);
                acc_q.mul(acc_q_temp,gyro_q_inv);
                accfinal[0]=(float)acc_q.x;
                accfinal[1]=(float)acc_q.y;
                accfinal[2]=(float)acc_q.z;
                Log.d("Acc", String.valueOf(accRunner));
                System.arraycopy(accfinal, 0, feature, 3 + 6 * accRunner, 3);
                Log.d("Acc2", String.valueOf(3+6*accRunner));
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
            ((TextView) findViewById(R.id.rotateValueX)).setText(Float.valueOf(event.values[0]).toString());
            ((TextView) findViewById(R.id.rotateValueY)).setText(Float.valueOf(event.values[1]).toString());
            ((TextView) findViewById(R.id.rotateValueZ)).setText(Float.valueOf(event.values[2]).toString());

            if(gyroRunner<72) {
             gyro[0]=event.values[0]-event.values[3];
             gyro[1]=event.values[1]-event.values[4];
             gyro[2]=event.values[2]-event.values[5];
/////////////////////////
    /*         gyro[0]=preGyroX[gyroRunner];
             gyro[1]=preGyroY[gyroRunner];
             gyro[2]=preGyroZ[gyroRunner];
*/
             ////////////////////////////////////////////
             gyro_uncal_q= new Quat4d(gyro[0],gyro[1],gyro[2],0);
             gyro_q_inv.inverse(gyro_uncal_q);
             gyro_q_temp.mul(gyro_uncal_q,ori_q);
             gyro_q.mul(gyro_q_temp,gyro_q_inv);
             gyrofinal[0]=(float)gyro_q.x;
             gyrofinal[1]=(float)gyro_q.y;
             gyrofinal[2]=(float)gyro_q.z;
                System.arraycopy(gyrofinal, 0, feature, 0 + 6 * gyroRunner, 3);
                gyroRunner++;
            }else if (gyroRunner==72 && accRunner==72){
                useNN();
                accRunner=0;
                gyroRunner=0;
            }

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
            //String file= String.valueOf(loadFile());
            //String file=assetFilePath("ronin_lstm_checkpoint");
            module = LiteModuleLoader.load(file,null, CPU);
            Log.d("model2",module.toString());
        } catch (Exception e) {
            Log.e("model", "Unable to load model for file", e);
        }

     /*   for(int i=0;i<72;i++) {
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
             Log.d("Ergebnis", res.toString());
             float[] x_and_y=res.getDataAsFloatArray();
            this.position[0]=x_and_y[0];
            this.position[1]=x_and_y[1];
             Log.d("Ergebnise", Arrays.toString(x_and_y));





        //    System.arraycopy(gyrofinal, 0, feature, 0, 3);
        //    System.arraycopy(accfinal, 0, feature, 3, 3);

        /*    Tensor inTensor2 = Tensor.fromBlob(feature,shape2);
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
        AssetFileDescriptor fileDescriptor=this.getAssets().openFd("model.tflite");
        FileInputStream inputStream=new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel=inputStream.getChannel();
        long startOffset=fileDescriptor.getStartOffset();
        long declareLength=fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY,startOffset,declareLength);
    }



}


