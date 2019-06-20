#include <SNPE/SNPE.hpp>
#include <SNPE/SNPEBuilder.hpp>
#include <SNPE/SNPEFactory.hpp>
#include <DlContainer/IDlContainer.hpp>
#include <DlSystem/DlError.hpp>
#include <DlSystem/ITensor.hpp>
#include <DlSystem/ITensorFactory.hpp>
#include <DlSystem/IUserBuffer.hpp>
#include <DlSystem/IUserBufferFactory.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

extern "C"{
  float run_model(float v_ego, float v_lead, float x_lead, float a_lead);
  void init_model();
}

struct testvar_t {
  float testvar;
};