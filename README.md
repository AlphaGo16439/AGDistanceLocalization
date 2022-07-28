# FTCDistanceSensorLocalization
A simple and easy-to-use library, written in Kotlin, to aid in the process of localizing your robot using Distance Sensors!
This library contains all the necessary calculations to localize on the FIRST Tech Challenge field in a "plug and play" style format!

### Advantages of this library
* Efficient position estimation, irrespective of robot heading or sensor location!
* Included filters to reduce sensor noise!
* Support for the RoadRunner library!
* Seamless integration into your code base!

### To learn how to get setup, please visit our [online documentation](https://alphago.gitbook.io/agdistancelocalization/)!
___

### Installation Instructions:

[![](https://jitpack.io/v/San68bot/FTCDistanceSensorLocalization.svg)](https://jitpack.io/#San68bot/FTCDistanceSensorLocalization)

In your ``build.dependencies.gradle`` file, add jitpack into repositories section:

```
repositories {
    ...
    maven { url 'https://jitpack.io' }
}
```

Then, add this into dependencies:

```
dependencies {
    ...
    implementation 'com.github.AlphaGo16439:AGDistanceLocalization:v1.0.0'
}
```