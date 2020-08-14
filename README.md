# Navtech IA SDK

The Navtech IA SDK provides a simple interface to communicate with the IA sensor. The SDK provides the source code for C++ and a simple .NET DLL that can easily be integrated into applications running on Windows and Linux.

The IA sensor operates in two modes, native data and plot extraction. The SDK provides support for both modes, allowing developers to receive either type of data from the sensor and also send control messages back.

Communication between the sensor and the software is over Ethernet and utilises a proprietary binary communication protocol called _Colossus Network Protocol_. The SDK abstracts this protocol to avoid having to develop the low level socket and messaging processing code. The [Colossus Protocol documentation can be found here](https://navtechradar.atlassian.net/wiki/display/PROD/Colossus+Network+Data+Protocol).

## SDK Requirements

### C++

* C++11 Compiler
* GCC 4.8 and above
* clang 3.5 and above
* Visual Studio 2019 (VC++ 2019 runtime libraries)

### Microsoft .NET

* .NET 4.8 and above

## Linux Requirements

To use the shell scripts provided we require bash on Ubuntu. The safest thing to do is execute:

```bash
sudo dpkg-reconfigure -p critical dash
```

## License

The iasdk which is released under The MIT License (MIT).
See file LICENSE.txt or go to <https://opensource.org/licenses/MIT> for full license details.

## C# Radar Client API

The .NET API is based on C#6 (.NET Framework 4.8) and was developed in Visual Studio 2019.
There are two project within the repro:

1. **IASDK** - The API DLL for use within any 3rd party projects to assist with connecting to the radar
1. **TestClient** - This is a very simple console application that runs up, connects to a radar and then displays some information before auto-disconnecting and closing. This provides a simple example of the recommended steps to connect and consume data from the radar.


The steps involved in connecting and getting data are as follows:

Setup your radar client and hook up the message and connection events:

```csharp
//Create the radar client and hook-up events
_radarTcpClient = new RadarTcpClient();
_radarTcpClient.OnConfigurationData += ConfigurationDataHandler;
_radarTcpClient.OnFftData += FftDataHandler;
_radarTcpClient.OnConnectionChanged += ConnectionChangedHandler;
```

Connect to the radar:

```csharp
//Connect to the radar
_radarTcpClient.Connect("192.168.0.1");
```

On successful connection you will receive a Configuration message with details of the radar's current configuration. So ensure you have the handler setup before you connect.

```csharp
//Configuration message handler
private void ConfigurationDataHandler(object sender, GenericEventArgs<TcpConfigurationDataMessage> configurationMessage)
{
   //Configure you client code to handle data based on the radar's configuration details
   //you receive here. i.e. getting rotation speed in Hz:
   var rotationHz = configurationMessage.Payload.RotationSpeed / 1000.0
}
```

Once connected and you have the config data, tell the radar to start sending FFT Data:

```csharp
//Tell the radar to start sending FFT Data
_radarTcpClient.StartFftData();
```

Ensure you handle incoming FFT Data:

```csharp
//FFT Data message handler
private static void FftDataHandler(object sender, GenericEventArgs<FftData> fftEventArgs)
{
   //Handle the FFT Data and do what you need. Each packet includes the azimuth and the signal amplitude for each range bin.
   //For example to get access to azimuth:
   var azimuth = fftEventArgs.Payload.Message.Azimuth;
   //Note that the azimuth is reported in encoder counts. The total encoder count is included in the radar config data. This can be used to convert the 
   //azimuth encoder reading to a bearing
}
```

When you need to disconnect, firstly stop the FFT Data:

```csharp
//Tell radar to stop sending FFT Data
_radarTcpClient.StopFftData();
```

Then disconnect:

```csharp
//Disconnect the client from the radar
_radarTcpClient.Disconnect();
```