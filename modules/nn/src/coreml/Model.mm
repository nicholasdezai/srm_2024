#include <Foundation/Foundation.h>
#if !__has_feature(objc_arc)
#error This file must be compiled with automatic reference counting enabled (-fobjc-arc)
#endif

#import "Model.h"

@implementation ModelInput

- (instancetype)initWithImage:(CVPixelBufferRef)image {
  self = [super init];
  if (self) {
    _image = image;
    CVPixelBufferRetain(_image);
  }
  return self;
}

- (void)dealloc {
  CVPixelBufferRelease(_image);
}

- (NSSet<NSString *> *)featureNames {
  return [NSSet setWithArray:@[ _inputName ]];
}

- (nullable MLFeatureValue *)featureValueForName:(NSString *)featureName {
  if ([featureName isEqualToString:_inputName]) {
    return [MLFeatureValue featureValueWithPixelBuffer:self.image];
  }
  return nil;
}

@end

@implementation ModelOutput

- (instancetype)initWithOut:(MLMultiArray *)out {
  self = [super init];
  if (self) {
    _out = out;
  }
  return self;
}

- (NSSet<NSString *> *)featureNames {
  return [NSSet setWithArray:@[ _outputName ]];
}

- (nullable MLFeatureValue *)featureValueForName:(NSString *)featureName {
  if ([featureName isEqualToString:_outputName]) {
    return [MLFeatureValue featureValueWithMultiArray:self.out];
  }
  return nil;
}

@end

@implementation Model

- (instancetype)initWithMLModel:(MLModel *)model {
  self = [super init];
  if (!self) {
    return nil;
  }
  _model = model;
  if (_model == nil) {
    return nil;
  }
  _inputName =
      model.modelDescription.inputDescriptionsByName.allKeys.firstObject;
  _outputName =
      model.modelDescription.outputDescriptionsByName.allKeys.firstObject;
  MLModelDescription *modelDescription = [_model modelDescription];
  NSDictionary<NSString *, MLFeatureDescription *> *inputDescriptions =
      [modelDescription inputDescriptionsByName];
  _inputH =
      ((MLImageConstraint *)[inputDescriptions[_inputName] imageConstraint])
          .pixelsHigh;
  _inputW =
      ((MLImageConstraint *)[inputDescriptions[_inputName] imageConstraint])
          .pixelsWide;
  return self;
}

- (nullable instancetype)
    initWithContentsOfURL:(NSURL *)modelURL
                    error:(NSError *_Nullable __autoreleasing *_Nullable)error {
  MLModel *model = [MLModel modelWithContentsOfURL:modelURL error:error];
  if (model == nil) {
    return nil;
  }
  return [self initWithMLModel:model];
}

- (nullable instancetype)
    initWithContentsOfURL:(NSURL *)modelURL
            configuration:(MLModelConfiguration *)configuration
                    error:(NSError *_Nullable __autoreleasing *_Nullable)error {
  MLModel *model = [MLModel modelWithContentsOfURL:modelURL
                                     configuration:configuration
                                             error:error];
  if (model == nil) {
    return nil;
  }
  return [self initWithMLModel:model];
}

- (nullable ModelOutput *)
    predictionFromFeatures:(ModelInput *)input
                     error:
                         (NSError *_Nullable __autoreleasing *_Nullable)error {
  return [self predictionFromFeatures:input
                              options:[[MLPredictionOptions alloc] init]
                                error:error];
}

- (nullable ModelOutput *)
    predictionFromFeatures:(ModelInput *)input
                   options:(MLPredictionOptions *)options
                     error:
                         (NSError *_Nullable __autoreleasing *_Nullable)error {
  id<MLFeatureProvider> outFeatures =
      [self.model predictionFromFeatures:input options:options error:error];
  if (!outFeatures) {
    return nil;
  }
  ModelOutput *output_ = [[ModelOutput alloc]
      initWithOut:(MLMultiArray *)[outFeatures featureValueForName:_outputName]
                      .multiArrayValue];
  output_.outputName = _outputName;
  return output_;
}

- (nullable ModelOutput *)
    predictionFromImage:(CVPixelBufferRef)image
                  error:(NSError *_Nullable __autoreleasing *_Nullable)error {
  ModelInput *input_ = [[ModelInput alloc] initWithImage:image];
  input_.inputName = _inputName;
  return [self predictionFromFeatures:input_ error:error];
}
@end