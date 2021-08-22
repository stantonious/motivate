
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
  const int kModelArenaSize = 3120;
  // Extra headroom for model + alignment + future interpreter changes.
  const int kExtraArenaSize = 560 + 16 + 100;
  const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
  static uint8_t tensor_arena[kTensorArenaSize];
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
  float dequant[7] = {
      (output->data.int8[0] - output->params.zero_point) * output->params.scale,
      (output->data.int8[1] - output->params.zero_point) * output->params.scale,
      (output->data.int8[2] - output->params.zero_point) * output->params.scale,
      (output->data.int8[3] - output->params.zero_point) * output->params.scale,
      (output->data.int8[4] - output->params.zero_point) * output->params.scale,
      (output->data.int8[5] - output->params.zero_point) * output->params.scale,
      (output->data.int8[6] - output->params.zero_point) * output->params.scale,
  };

  float max_conf = get_max(dequant, 7);

  if (max_conf < thresh)
  {
    //ESP_LOGI(TAG, "confidence too low return 0");
    return -1;
  }

  //ESP_LOGI(TAG, "BWS ============== out 1 %f", dequant[0]);
  //ESP_LOGI(TAG, "BWS ============== out 2 %f", dequant[1]);
  //ESP_LOGI(TAG, "BWS ============== out 3 %f", dequant[2]);
  return get_max_idx(dequant, 7);
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
  float dequant[7] = {
      (output->data.int8[0] - output->params.zero_point) * output->params.scale,
      (output->data.int8[1] - output->params.zero_point) * output->params.scale,
      (output->data.int8[2] - output->params.zero_point) * output->params.scale,
      (output->data.int8[3] - output->params.zero_point) * output->params.scale,
      (output->data.int8[4] - output->params.zero_point) * output->params.scale,
      (output->data.int8[5] - output->params.zero_point) * output->params.scale,
      (output->data.int8[6] - output->params.zero_point) * output->params.scale,
  };

  float max_conf = get_max(dequant, 7);

  if (max_conf < thresh)
  {
    //ESP_LOGI(TAG, "confidence too low return 0");
    return -1;
  }

  //ESP_LOGI(TAG, "BWS ============== out 1 %f", dequant[0]);
  //ESP_LOGI(TAG, "BWS ============== out 2 %f", dequant[1]);
  //ESP_LOGI(TAG, "BWS ============== out 3 %f", dequant[2]);
  return get_max_idx(dequant, 7);
}
