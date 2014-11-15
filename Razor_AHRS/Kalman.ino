//Taken from http://greg.czerniak.info/guides/kalman1/kalman1.py.txt
float Step(float control_vector[1], float measurement_vector, float current_state_estimate, float current_prob_estimate){
  //--------------------Prediction Step-------------------------
  float predicted_state_estimate = A[0] * current_state_estimate + B[0] * control_vector[0];
  float predicted_prob_estimate = (A[0] * current_prob_estimate) * A[0] + Q[0]; 
  //Serial.println(predicted_state_estimate);
  //Serial.println(predicted_prob_estimate);
  //--------------------Observation Step-------------------------
  float innovation = measurement_vector - H[0] * predicted_state_estimate;
  float innovation_covariance = H[0] * predicted_prob_estimate * H[0] + R[0];
  
  //--------------------Update Step-------------------------
  float kalman_gain = predicted_prob_estimate * H[0] * (1.0/innovation_covariance);
  current_state_estimate = predicted_state_estimate + kalman_gain * innovation;
  current_prob_estimate = (1.0 - kalman_gain * H[0]) * predicted_prob_estimate;
  return current_state_estimate, current_prob_estimate;
}
