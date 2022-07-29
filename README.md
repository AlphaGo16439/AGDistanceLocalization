# FTCDistanceSensorLocalization
A simple and easy-to-use library, written in Kotlin, to aid in the process of localizing your robot using Distance Sensors!
This library contains all the necessary calculations to localize on the FIRST Tech Challenge field in a "plug and play" style format!

### Advantages of this library
* Efficient position estimation, irrespective of robot heading and sensor location!
* Accounts for the ['corner cases problem'](https://alphago.gitbook.io/agdistancelocalization/the-corner-case-problem), where all sensors are in use!
* Support for the RoadRunner library!
* Seamless integration into your code base!

### Coming soon to this library
* Four Sensor Localization!

### To learn how to get setup, please visit our [online documentation](https://alphago.gitbook.io/agdistancelocalization/)!
___

### Installation Instructions:

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