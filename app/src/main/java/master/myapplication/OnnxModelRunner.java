package master.myapplication;
/*
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;

public class OnnxModelRunner {

    private OrtEnvironment env;
    private OrtSession session;

    public OnnxModelRunner(String modelPath) throws OrtException {
        env = OrtEnvironment.getEnvironment();
        session = env.createSession(modelPath);
    }

    /public float[] runModel(float[] inputArray) throws OrtException {
        OnnxTensor inputTensor = OnnxTensor.createTensor(env, inputArray);
        OnnxTensor outputTensor = (OnnxTensor) session.run(inputTensor);
        float[] result = outputTensor.getFloatArray();
        inputTensor.close();
        outputTensor.close();
        return result;
    }

    public void close() {
        session.close();
        env.close();
    }
}*/