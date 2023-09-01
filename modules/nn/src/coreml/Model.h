#ifndef _SRM_NN_MODEL_H_
#define _SRM_NN_MODEL_H_
#import <CoreML/CoreML.h>
#import <Foundation/Foundation.h>
#include <objc/NSObjCRuntime.h>

NS_ASSUME_NONNULL_BEGIN

API_AVAILABLE(macos(10.15), ios(13.0), watchos(6.0), tvos(13.0))
__attribute__((visibility("hidden")))
@interface ModelInput : NSObject<MLFeatureProvider>

@property(readwrite, nonatomic) CVPixelBufferRef image;
@property(readwrite, nonatomic) NSString *inputName;
- (instancetype)init NS_UNAVAILABLE;
- (instancetype)initWithImage:(CVPixelBufferRef)image NS_DESIGNATED_INITIALIZER;
@end

API_AVAILABLE(macos(10.15), ios(13.0), watchos(6.0), tvos(13.0))
__attribute__((visibility("hidden")))
@interface ModelOutput : NSObject<MLFeatureProvider>

@property(readwrite, nonatomic, strong) MLMultiArray *out;
@property(readwrite, nonatomic) NSString *outputName;
- (instancetype)init NS_UNAVAILABLE;
- (instancetype)initWithOut:(MLMultiArray *)out NS_DESIGNATED_INITIALIZER;
@end

API_AVAILABLE(macos(10.15), ios(13.0), watchos(6.0), tvos(13.0))
__attribute__((visibility("hidden")))
@interface Model : NSObject
@property(readonly, nonatomic, nullable) MLModel *model;
@property(readwrite, nonatomic) NSString *inputName;
@property(readwrite, nonatomic) NSString *outputName;
@property(readwrite, nonatomic) NSInteger inputW;
@property(readwrite, nonatomic) NSInteger inputH;

- (instancetype)initWithMLModel:(MLModel *)model;

- (nullable instancetype)initWithContentsOfURL:(NSURL *)modelURL
                                         error:(NSError *_Nullable __autoreleasing *_Nullable)error;
- (nullable instancetype)initWithContentsOfURL:(NSURL *)modelURL
                                 configuration:(MLModelConfiguration *)configuration
                                         error:(NSError *_Nullable __autoreleasing *_Nullable)error;
- (nullable ModelOutput *)predictionFromFeatures:(ModelInput *)input
                                           error:(NSError *_Nullable __autoreleasing *_Nullable)error;
- (nullable ModelOutput *)predictionFromFeatures:(ModelInput *)input
                                         options:(MLPredictionOptions *)options
                                           error:(NSError *_Nullable __autoreleasing *_Nullable)error;
- (nullable ModelOutput *)predictionFromImage:(CVPixelBufferRef)image
                                        error:(NSError *_Nullable __autoreleasing *_Nullable)error;
@end

NS_ASSUME_NONNULL_END

#endif  //_SRM_NN_MODEL_H_