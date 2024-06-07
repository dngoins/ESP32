using Microsoft.CognitiveServices.Speech;

public class SpeechConfigSingleton
{
    private static SpeechConfigSingleton _instance;
    public SpeechConfig Config { get; private set; }

    private SpeechConfigSingleton()
    {
        // Your setup code here
        string speechKey = Environment.GetEnvironmentVariable("SPEECH_SUB_KEY");
        string speechRegion = Environment.GetEnvironmentVariable("SPEECH_REGION");
        if (string.IsNullOrEmpty(speechRegion))
            speechRegion = "eastus";
        bool useFromSubscription = false;
        string speechUrl = Environment.GetEnvironmentVariable("SPEECH_ENDPOINT");
        if (string.IsNullOrEmpty(speechUrl))
            useFromSubscription = true;
        if (useFromSubscription)
        {
            Config = SpeechConfig.FromSubscription(speechKey, speechRegion);
        }
        else
        {
            Uri endpoint = new Uri(speechUrl);
            Config = SpeechConfig.FromEndpoint(endpoint, speechKey);
        }
        bool enableAudioLogging = false;
        bool.TryParse(Environment.GetEnvironmentVariable("ENABLE_SPEECH_LOGGING"), out enableAudioLogging);
        if (enableAudioLogging)
        {
            var currentTimeStamp = DateTime.UtcNow.ToString("yyyyMMddHHmmss");
            Config.SetProperty(PropertyId.Speech_LogFilename, $".\\{currentTimeStamp}-SpeechSDK.logs");
            Config.EnableAudioLogging();
        }
        Config.SetProfanity(ProfanityOption.Masked);

    }

    public static SpeechConfigSingleton Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = new SpeechConfigSingleton();
            }
            return _instance;
        }
    }
}
