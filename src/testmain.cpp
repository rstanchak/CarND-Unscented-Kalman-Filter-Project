void test_UpdateState( void );
void test_AugmentedSigmaPoints( void );
void test_PredictMeanAndCovariance( void );
void test_SigmaPointPrediction( void );
void test_PredictRadarMeasurement( void );

int main(int argc, char ** argv)
{
    test_AugmentedSigmaPoints();
    test_SigmaPointPrediction();
    test_PredictMeanAndCovariance();
    test_PredictRadarMeasurement();
    test_UpdateState();
}
