
#include "mot-imu-tf.h"
#include "esp_log.h"
#include <stdio.h>
#include "float_buffer.h"
#include "CircularBuffer.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "constants.h"
#include "mot-imu-model.h"
//#include "output_handler.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

static const char *TAG = "MOT_IMU_TFL_HANDLER";

/*
static float test3[10][6] = {

    {0.0185546875, 0.173828125, 0.07470703125, 3.662109375, -50.6591796875, -112.4267578125},
    {-0.23095703125, 0.202880859375, 0.198486328125, -4.39453125, -41.44287109375, -78.18603515625},
    {-0.305908203125, 0.198486328125, 0.041015625, -9.21630859375, -15.07568359375, -37.109375},
    {-0.15673828125, 0.2685546875, -0.041748046875, -18.310546875, -1.953125, -21.728515625},
    {-0.02392578125, 0.2744140625, 0.0712890625, -10.68115234375, -5.37109375, -2.9296875},
    {0.049072265625, 0.242919921875, 0.09326171875, -14.34326171875, -8.544921875, -9.46044921875},
    {0.07470703125, 0.3134765625, 0.01416015625, -10.92529296875, 0.67138671875, -13.48876953125},
    {0.213134765625, 0.263671875, 0.059326171875, -5.31005859375, -15.56396484375, -43.3349609375},
    {0.2509765625, 0.155517578125, 0.09130859375, -12.87841796875, -31.31103515625, -93.8720703125},
    {0.198486328125, 0.0986328125, 0.147216796875, -10.07080078125, -67.07763671875, -137.5732421875},
};
static float test1[10][6] = {

    {0.037353515625, 0.326416015625, -0.00634765625, -10.55908203125, -4.2724609375, -4.8828125},
    {0.020263671875, 0.325439453125, -0.006103515625, -12.26806640625, -2.50244140625, -1.8310546875},
    {0.008544921875, 0.33154296875, -0.0146484375, -15.31982421875, -8.11767578125, 1.89208984375},
    {-0.01220703125, 0.37646484375, -0.048583984375, -19.6533203125, -3.7841796875, 2.13623046875},
    {0.004638671875, 0.38330078125, 0.009033203125, -25.45166015625, -3.90625, -4.08935546875},
    {-0.006103515625, 0.445556640625, -0.0205078125, -15.44189453125, 1.5869140625, 3.84521484375},
    {-0.027587890625, 0.388427734375, 0.0537109375, -6.7138671875, -12.26806640625, 6.8359375},
    {-0.01708984375, 0.37744140625, -0.023681640625, -8.1787109375, -14.46533203125, -6.04248046875},
    {0.07958984375, 0.329345703125, 0.358642578125, -6.103515625, -0.6103515625, -8.72802734375},
    {0.102294921875, 0.156982421875, 0.0546875, -1.46484375, -24.35302734375, -20.751953125},
};

static float test2[10][6] = {
    {0.0185546875, 0.14453125, 0.085205078125, -12.75634765625, -3.173828125, 0},
    {.00244140625, 0.142578125, 0.0810546875, -9.94873046875, -6.4697265625, 0},
    {.000732421875, 0.126708984375, 0.011474609375, -5.859375, 1.03759765625, -4.638671875},
    {.020751953125, 0.123291015625, 0.037353515625, -9.58251953125, -5.67626953125, -3.72314453125},
    {.0244140625, 0.133544921875, 0.06982421875, -5.9814453125, -3.7841796875, -2.13623046875},
    {0.013427734375, 0.13916015625, 0.08203125, -8.544921875, -5.79833984375, 1.46484375},
    {0.077880859375, 0.13623046875, 0.10302734375, -10.07080078125, -7.38525390625, 14.83154296875},
    {0.19140625, 0.13232421875, 0.104248046875, -0.91552734375, -1.64794921875, 49.49951171875},
    {0.2080078125, 0.100830078125, 0.060791015625, -0.8544921875, 18.61572265625, 95.64208984375},
    {0.12646484375, 0.085693359375, 0.063720703125, -4.7607421875, 14.892578125, 108.58154296875},
};
*/

static TfLiteTensor *input = nullptr;
static TfLiteTensor *output = nullptr;
static const int g_min = -90;
static const int g_max = 90;
static const int a_min = -1;
static const int a_max = 1;
// Globals, used for compatibility with Arduino-style sketches.
namespace
{
  tflite::ErrorReporter *error_reporter = nullptr;
  const tflite::Model *model = nullptr;
  tflite::MicroInterpreter *interpreter = nullptr;

  // Create an area of memory to use for input, output, and intermediate arrays.
  // Minimum arena size, at the time of writing. After allocating tensors
  // you can retrieve this value by invoking interpreter.arena_used_bytes().
  const int kModelArenaSize = 27208;
  // Extra headroom for model + alignment + future interpreter changes.
  const int kExtraArenaSize = 560 + 16 + 100;
  const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
  static uint8_t tensor_arena[3124];
} // namespace

void init_mot_imu(void)
{
  ESP_LOGI(TAG,"INITING Model");
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  //static tflite::AllOpsResolver resolver;
  static tflite::MicroMutableOpResolver<5> resolver;
  resolver.AddConv2D();
  resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
             tflite::ops::micro::Register_MAX_POOL_2D());
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  resolver.AddReshape();

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;


  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  ESP_LOGI(TAG,"Arena size %d",static_interpreter.arena_used_bytes());
  if (allocate_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }
  input = interpreter->input(0);
  output = interpreter->output(0);
  /*
  */
  //xTaskCreatePinnedToCore(mot_imu_task, "MotImuTfTask", 2096, NULL, 1, &mot_imu_handle, 0);
}

float get_max(float *f, int n)
{
  float max = f[0];
  for (int i = 1; i < n; i++)
  {
    if (f[i] > max)
      max = f[i];
  }
  return max;
}
int get_max_idx(float *f, int n)
{
  int max_idx = 0;
  float max = f[0];
  for (int i = 1; i < n; i++)
  {
    if (f[i] > max)
    {
      max_idx = i;
      max = f[i];
    }
  }
  return max_idx;
}
int infer(float **a_samples, float **g_samples, int a_size, int g_size)
{

  // Place our calculated x value in the model's input tensor
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int8_t quant_val = (a_samples[j][i] - a_min) / (a_max - a_min) * 255 / input->params.scale + input->params.zero_point;
      int8_t *d_i = tflite::GetTensorData<int8>(input);
      d_i[i * 6 + j] = quant_val;
      //input->data.uint8[i * 6 + j] = (test[i][j]-a_min)/(a_max-a_min) * 255;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    for (int j = 3; j < 6; j++)
    {
      int8_t quant_val = (g_samples[j - 3][i] - g_min) / (g_max - g_min) * 255 / input->params.scale + input->params.zero_point;
      int8_t *d_i = tflite::GetTensorData<int8>(input);
      d_i[i * 6 + j] = quant_val;
      //input->data.uint8[i * 6 + j] = (test[i][j]-g_min)/(g_max-g_min) * 255;
    }
  }

  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x_val: %f\n", 0.);
    return -1;
  }

  static float thresh = .6;
  // Read the predicted y value from the model's output tensor
  float dequant[6] = {
      (output->data.int8[0] - output->params.zero_point) * output->params.scale,
      (output->data.int8[1] - output->params.zero_point) * output->params.scale,
      (output->data.int8[2] - output->params.zero_point) * output->params.scale,
      (output->data.int8[3] - output->params.zero_point) * output->params.scale,
      (output->data.int8[4] - output->params.zero_point) * output->params.scale,
      (output->data.int8[5] - output->params.zero_point) * output->params.scale,
  };

  float max_conf = get_max(dequant, 6);

  if (max_conf < thresh)
  {
    //ESP_LOGI(TAG, "confidence too low return 0");
    return -1;
  }

  //ESP_LOGI(TAG, "BWS ============== out 1 %f", dequant[0]);
  //ESP_LOGI(TAG, "BWS ============== out 2 %f", dequant[1]);
  //ESP_LOGI(TAG, "BWS ============== out 3 %f", dequant[2]);
  return get_max_idx(dequant, 6);
}

int buffer_infer(void *ax,
                 void *ay,
                 void *az,
                 void *gx,
                 void *gy,
                 void *gz)
{

  CircularBuffer<float, BUFSIZE> *ax_cb = (CircularBuffer<float, BUFSIZE> *)ax;
  CircularBuffer<float, BUFSIZE> *ay_cb = (CircularBuffer<float, BUFSIZE> *)ay;
  CircularBuffer<float, BUFSIZE> *az_cb = (CircularBuffer<float, BUFSIZE> *)az;
  CircularBuffer<float, BUFSIZE> *gx_cb = (CircularBuffer<float, BUFSIZE> *)gx;
  CircularBuffer<float, BUFSIZE> *gy_cb = (CircularBuffer<float, BUFSIZE> *)gy;
  CircularBuffer<float, BUFSIZE> *gz_cb = (CircularBuffer<float, BUFSIZE> *)gz;

  int8_t *d_i = tflite::GetTensorData<int8>(input);

  // Place our calculated x value in the model's input tensor
  for (int i = 0; i < 10; i++)
  {
    int8_t quant_val = ((*ax_cb)[i] - a_min) / (a_max - a_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 0] = quant_val;

    quant_val = ((*ay_cb)[i] - a_min) / (a_max - a_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 1] = quant_val;

    quant_val = ((*az_cb)[i] - a_min) / (a_max - a_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 2] = quant_val;

    quant_val = ((*gx_cb)[i] - g_min) / (g_max - g_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 3] = quant_val;

    quant_val = ((*gy_cb)[i] - g_min) / (g_max - g_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 4] = quant_val;

    quant_val = ((*gz_cb)[i] - g_min) / (g_max - g_min) * 255 / input->params.scale + input->params.zero_point;
    d_i[i * 6 + 5] = quant_val;
  }

  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x_val: %f\n", 0.);
    return -1;
  }

  static float thresh = .6;
  // Read the predicted y value from the model's output tensor
  float dequant[6] = {
      (output->data.int8[0] - output->params.zero_point) * output->params.scale,
      (output->data.int8[1] - output->params.zero_point) * output->params.scale,
      (output->data.int8[2] - output->params.zero_point) * output->params.scale,
      (output->data.int8[3] - output->params.zero_point) * output->params.scale,
      (output->data.int8[4] - output->params.zero_point) * output->params.scale,
      (output->data.int8[5] - output->params.zero_point) * output->params.scale,
  };

  float max_conf = get_max(dequant, 6);

  if (max_conf < thresh)
  {
    //ESP_LOGI(TAG, "confidence too low return 0");
    return -1;
  }

  //ESP_LOGI(TAG, "BWS ============== out 1 %f", dequant[0]);
  //ESP_LOGI(TAG, "BWS ============== out 2 %f", dequant[1]);
  //ESP_LOGI(TAG, "BWS ============== out 3 %f", dequant[2]);
  return get_max_idx(dequant, 6);
}
