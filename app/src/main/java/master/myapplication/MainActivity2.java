package master.myapplication;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;

import org.pytorch.IValue;
import org.pytorch.LiteModuleLoader;
import org.pytorch.Module;
import org.pytorch.Tensor;
import org.pytorch.torchvision.TensorImageUtils;
import org.pytorch.MemoryFormat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import androidx.appcompat.app.AppCompatActivity;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;


public class MainActivity2 extends AppCompatActivity {
    private static final int INPUT_WIDTH = 244;
    private static final int INPUT_HEIGHT = 244;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity2_main);

        Bitmap bitmap = null;
        Module module = null;
        try {
            // creating bitmap from packaged into app android asset 'image.jpg',
            // app/src/main/assets/image.jpg
            bitmap = BitmapFactory.decodeStream(getAssets().open("image.jpg"));
            // loading serialized torchscript module from packaged into app android asset model.pt,
            module = LiteModuleLoader.load(assetFilePath(this, "resnet_mobile.ptl"));

        } catch (IOException e) {
            Log.e("PytorchHelloWorld", "Error reading assets", e);
            finish();
        }

        // showing image on UI
        ImageView imageView = findViewById(R.id.image);
        imageView.setImageBitmap(bitmap);

        // preparing input tensor
        //final Tensor inputTensor = convertBitmapToTensor(bitmap);
        // running the model
        Tensor inputTensor = TensorImageUtils.bitmapToFloat32Tensor(bitmap,
                TensorImageUtils.TORCHVISION_NORM_MEAN_RGB, TensorImageUtils.TORCHVISION_NORM_STD_RGB);
        Log.d("Ergebnis",inputTensor.toString());
        inputTensor=removeFirstDimension(inputTensor);
        Log.d("Ergebnis",inputTensor.toString());
        Log.d("Ergebnis",Arrays.toString(inputTensor.shape()));
        final Tensor outputTensor = module.forward(IValue.from(inputTensor)).toTensor();
        // getting tensor content as java array of floats
        final float[] scores = outputTensor.getDataAsFloatArray();
        Log.d("Ergebnis", outputTensor.toString());

        // searching for the index with maximum score
        float maxScore = -Float.MAX_VALUE;
        int maxScoreIdx = -1;
        for (int i = 0; i < scores.length; i++) {
            if (scores[i] > maxScore) {
                maxScore = scores[i];
                maxScoreIdx = i;
            }
        }

        String className = ImageNetClasses.IMAGENET_CLASSES[maxScoreIdx];

        // showing className on UI
        TextView textView = findViewById(R.id.text);
        textView.setText(className);


    }

    /**
     * Copies specified asset to the file in /files app directory and returns this file absolute path.
     *
     * @return absolute file path
     */
    public static String assetFilePath(Context context, String assetName) throws IOException {
        File file = new File(context.getFilesDir(), assetName);
        if (file.exists() && file.length() > 0) {
            return file.getAbsolutePath();
        }
        try (InputStream is = context.getAssets().open(assetName)) {
            try (OutputStream os = new FileOutputStream(file)) {
                byte[] buffer = new byte[4 * 1024];
                int read;
                while ((read = is.read(buffer)) != -1) {
                    os.write(buffer, 0, read);
                }
                os.flush();
            }
            return file.getAbsolutePath();
        }
    }

        public Tensor convertBitmapToTensor(Bitmap bitmap) {
            // Resize the bitmap to match the input size of the model
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bitmap, INPUT_WIDTH, INPUT_HEIGHT, true);

            // Flatten the pixel values to a 1D float array
            float[] inputTensor = convertBitmapToFloatArray(resizedBitmap);
            // Convert the input tensor to a PyTorch tensor
            System.out.println(Arrays.toString(inputTensor));
            System.out.println(inputTensor.length);
            Tensor inputPyTorchTensor = Tensor.fromBlob(inputTensor, new long[]{244,244,3});

            return inputPyTorchTensor;

        }

    public static float[] convertBitmapToFloatArray(Bitmap bitmap) {
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int channels = 3;  // RGB

        // Calculate the total number of pixels
        int size = width * height * channels;

        // Create a float array to store the pixel values
        float[] floatArray = new float[size];

        // Iterate through each pixel and convert it to a float value
        int[] pixels = new int[size / channels];
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
        System.out.println(pixels.length);
        int floatArrayIndex = 0;
        for (int i = 0; i < pixels.length; i++) {
            int pixelValue = pixels[i];

            // Extract the red, green, and blue channel values
            int red = (pixelValue >> 16) & 0xFF;
            int green = (pixelValue >> 8) & 0xFF;
            int blue = pixelValue & 0xFF;

            // Convert the channel values to floats between 0 and 1
            floatArray[floatArrayIndex++] = red / 255.0f;
            floatArray[floatArrayIndex++] = green / 255.0f;
            floatArray[floatArrayIndex++] = blue / 255.0f;
        }

        return floatArray;
    }


    private static Tensor removeFirstDimension(Tensor inputTensor) {
        // Get the shape of the input tensor
        long[] inputShape = inputTensor.shape();

        // Create a new shape without the first dimension
        long[] newShape = Arrays.copyOfRange(inputShape, 1, inputShape.length);

        // Calculate the number of elements in the new tensor
        long numel = Tensor.numel(newShape);

        // Create a new tensor with the desired shape
        Tensor removedFirstDimensionTensor = Tensor.fromBlob(new float[(int) numel], newShape);

        // Manually copy the data from the original tensor to the new tensor
        float[] originalData = inputTensor.getDataAsFloatArray();
        Log.d("shaper", String.valueOf(originalData.length));
        float[] newData = removedFirstDimensionTensor.getDataAsFloatArray();
        Log.d("shaper", String.valueOf(newData.length));

        int inputOffset = (int) inputShape[1];
        for (int i = 0; i < (numel-inputOffset); i++) {
            newData[i] = originalData[inputOffset + i];
        }
       // System.arraycopy(originalData, (int) inputShape[1], newData, 0, (int) numel);
        return removedFirstDimensionTensor;
    }

}

