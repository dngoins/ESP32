using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Configuration;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.SignalR;
using NAudio.Wave;
using NAudio.Wave.SampleProviders;
using Microsoft.CognitiveServices.Speech.Transcription;
using Microsoft.CognitiveServices.Speech;
using Microsoft.CognitiveServices.Speech.Audio;
using System.Text;
using Azure;
using Azure.AI.Language.Conversations;
using Azure.Core.Serialization;
using Azure.Core;
using System.Text.Json;
using System.ComponentModel;
using Microsoft.AspNetCore.SignalR.Protocol;
using System;
using Microsoft.Extensions.Azure;
using Azure.Storage.Blobs;
using NAudio.Codecs;

namespace azfunction
{
    public class message
    {
        private readonly ILogger<message> _logger;

        private SpeechConfig config = null;

        private float confidenceFactor = 0.75f;

        private const int FRAMESIZE = 160;

        private StringBuilder recognizedText = new StringBuilder();

        private List<string> TopIntent;
        private ConversationAnalysisClient client;

        private string CLU_ProjectName;
        private string CLU_DeploymentName;
        private static MemoryStream audioBuffer = new MemoryStream();
        private static DateTime lastReceived = DateTime.UtcNow;
        
        private static int currentFrameCount = 0;
        private BlobServiceClient blobServiceClient;
        private TaskCompletionSource<int> stopRecognition;
        private bool WriteToBlobStorage = false;

        public message(ILogger<message> logger)
        {
            _logger = logger;

            //Guid lsAppId = Guid.Parse(configuration["LSAppID"]);
            string predictionEndpoint = Environment.GetEnvironmentVariable("AIServicesEndpoint");
            string predictionKey = Environment.GetEnvironmentVariable("AIServicesKey");
            
            CLU_ProjectName = Environment.GetEnvironmentVariable("CLU_PROJECT_NAME");
            CLU_DeploymentName = Environment.GetEnvironmentVariable("CLU_DEPLOYMENT_NAME");

            bool.TryParse(Environment.GetEnvironmentVariable("SHOULD_WRITE_TO_BLOB"), out WriteToBlobStorage);

            Uri clu_endpoint = new Uri(predictionEndpoint);
            AzureKeyCredential credential = new AzureKeyCredential(predictionKey);

            client = new ConversationAnalysisClient(clu_endpoint, credential);
            config = SpeechConfigSingleton.Instance.Config;

            string storageConnectionString = Environment.GetEnvironmentVariable("AZURE_STORAGE_CONNECTION_STRING");
            blobServiceClient = new BlobServiceClient(storageConnectionString);

        }

        [Function("message")]
        [WebPubSubOutput(Hub = "notification")]
        public SendToAllAction Run(
            [WebPubSubTrigger("notification", WebPubSubEventType.User, "message")] UserEventRequest request)
        {

            if (request.ConnectionContext.UserId.ToLower() != "iot") return null;

            //_logger.LogInformation($"Request from: {request.ConnectionContext.UserId}");
            ////_logger.LogInformation($"Request message data: {request.Data}");
            //_logger.LogInformation($"Request message dataType: {request.DataType}");
            //// process binary data (assumed to be audio data)
            
            
            if (request.DataType == WebPubSubDataType.Binary)
            {
                // Get the data
                // Append the incoming audio data to the buffer.
                var binaryData = request.Data;
                byte[] audioData = binaryData.ToArray();

                audioBuffer.Write(audioData);

                currentFrameCount++;

                //_logger.LogInformation($"Binary message data");

                // lets just send in the audio as it comes in
                var now = DateTime.UtcNow;
                if ((Math.Abs ((now - lastReceived).TotalSeconds ) > 4) && (currentFrameCount > 10))
                {                    
                    _logger.LogInformation($"Processing audio data frame count: {currentFrameCount}");
                    currentFrameCount = 0;
                    // More than 8 seconds have passed since the last chunk of audio data was received.
                    // Process the buffered audio data here.
                    var result =  ProcessAudioData(audioBuffer.ToArray(), request);

                    // Clear the buffer.
                    audioBuffer.Position = 0;
                    audioBuffer.SetLength(0);                    
                    lastReceived = now;
                    return result;                    
                }

                return null;
            }
            else
            {
                _logger.LogInformation($"Request message data: {request.Data}");

                var command = request.Data.ToString().ToLower();
                if (command == "stop")
                {
                    _logger.LogInformation("Stopping Recognition");
                    if (stopRecognition != null)
                    {
                        var stopped = stopRecognition.TrySetResult(0);
                        if (stopped)
                        {
                            _logger.LogInformation("Recognition Stopped");
                        }
                    }
                    return null;
                }

                var prefix = command.Substring(0, 4);
                if ( prefix == "clu:")
                {
                    // strip off prefix
                    var text = command.Substring(4);
                    _logger.LogInformation($"Text: {text}");

                    // send to CLU
                    return CLU_IntentAction(text, request);
                }

                TopIntent = new List<string>();
                TopIntent.Add(request.Data.ToString());
                TopIntent.Add("");
            }

            _logger.LogInformation($"Top Intent: {TopIntent[0]}");
            return new SendToAllAction
            {
                Data = BinaryData.FromString($"[{request.ConnectionContext.UserId}] {TopIntent[0]} {TopIntent[1]}"),
                DataType = WebPubSubDataType.Text
            };
        }
      
        private SendToAllAction ProcessAudioData(byte[] audioData, UserEventRequest request)
        {
            // get the audioBuffer length
            var audioBufferLength = audioBuffer.Length;
            //_logger.LogInformation($"Audio data length: {audioData.Length}");
            //byte[] encoded = new byte[audioBufferLength / 2];
            //for (int i = 0; i < audioData.Length; i += 2)
            //{
            //    short sample = BitConverter.ToInt16(audioData, i);
            //    encoded[i / 2] = ALawEncoder.LinearToALawSample(sample);
            //}



            // now create a wav file from the audioBuffer
            var resampledAudioData = ResampleAudioStream(audioData, WriteToBlobStorage);
                _logger.LogInformation("Audio has been resampled");


                if (resampledAudioData != null)
                {
                    //Console.Write("Audio is not null");
                    _logger.LogInformation($"Transcribing Audio Stream");
                RecognitionWithPushAudioStreamAsync(resampledAudioData, resampledAudioData.Length).GetAwaiter().GetResult();
                }
                                
                var transcribedText = recognizedText.ToString();


            if (string.IsNullOrEmpty(transcribedText))
            {
                _logger.LogInformation("Nothing recognized");
                return null;
            }   

            _logger.LogInformation($"Transcription: {transcribedText}");
            return CLU_IntentAction(transcribedText, request);
        }

        private SendToAllAction CLU_IntentAction(string transcribedText, UserEventRequest request)
        {
            _logger.LogInformation($"Now let's call Conversation Language Understanding Azure AI Services to get the top intent");
            TopIntent = GetTopIntentAndEntitiesAsync(transcribedText).GetAwaiter().GetResult();


            while (TopIntent.Count < 2)
            {
                TopIntent.Add("");
            }

            recognizedText.Clear();

            _logger.LogInformation($"Top Intent: {TopIntent[0]}");
            return new SendToAllAction
            {
                Data = BinaryData.FromString($"[{request.ConnectionContext.UserId}] {TopIntent[0]} {TopIntent[1]}"),
                DataType = WebPubSubDataType.Text
            };
            
        }
        private async Task<List<string>> GetTopIntentAndEntitiesAsync(string transcribedText)
        {
            List<string> results = new List<string>();

            if (string.IsNullOrEmpty(transcribedText))
            {
                results.Add("Nothing recognized");
                //stopRecognition.TrySetResult(0);
                return results;
            }

            // create client to LUIS 
            // Call the Language service model to get intent and entities
            var projectName = CLU_ProjectName;
            var deploymentName = CLU_DeploymentName;

            //log proper projectName and deploymentName
            //_logger.LogInformation($"Project Name: {projectName}");
            //_logger.LogInformation($"Deployment Name: {deploymentName}");

            var data = new
            {
                analysisInput = new
                {
                    conversationItem = new
                    {
                        text = transcribedText,
                        id = "1",
                        participantId = "1",
                    }
                },
                parameters = new
                {
                    projectName,
                    deploymentName,
                    // Use Utf16CodeUnit for strings in .NET.
                    stringIndexType = "Utf16CodeUnit",
                },
                kind = "Conversation",
            };

            //_logger.LogInformation($"About to invoke CLU");

            try
            {
                // Send request
                Response response = await client.AnalyzeConversationAsync(RequestContent.Create(data));

                // log the CLU response
                // _logger.LogInformation($"CLU Response: {response.Content}");

                dynamic conversationalTaskResult = response.Content.ToDynamicFromJson(JsonPropertyNames.CamelCase);
                dynamic conversationPrediction = conversationalTaskResult.Result.Prediction;
                var options = new JsonSerializerOptions { WriteIndented = true };
                var userText = JsonSerializer.Serialize(conversationPrediction, options);
                string strUserText = JsonSerializer.Serialize(conversationPrediction);

                _logger.LogInformation(message: strUserText);

                var topIntent = "Nothing recognized";
                if (conversationPrediction.Intents[0].ConfidenceScore > 0.5)
                {
                    topIntent = conversationPrediction.TopIntent;
                }
                results.Add(topIntent);

                foreach (dynamic entity in conversationPrediction.Entities)
                {
                    results.Add(entity.Text);
                }

            }
            catch (Exception ex)
            {
                _logger.LogError(message: $"Error Occurred: {ex.Message}");
            }
           // stopRecognition.TrySetResult(0);
            return results;
        }
        /// <summary>
        /// Takes the audio stream and resamples it up to 16000hz, 8bits, 1 channel
        /// This is the required format for the Speech API.
        /// </summary>
        /// <param name="audioChunk">an 8000hz, 8bit, 1 channel formatted array of audio bytes</param>
        /// <returns>A Byte array consisting of 16000hz, 8bits, 1 channel formatted audio buffer</returns>
        public byte[] ResampleAudioStream(byte[] audioChunk, bool writeToFile)
        {
            byte[] resampledBytes = null;
            try
            {
                var fileName = string.Format("~{0}-{1}.wav", "iot", DateTime.Now.ToString("yyyyMMdd_hhmmss"));
                var fileNameOrig = fileName.Replace ("~","~orig-");

                var copyOrigBytes = new byte[audioChunk.Length - 1];
                Array.Copy(audioChunk, copyOrigBytes, audioChunk.Length - 1);
                audioChunk = copyOrigBytes;

                // Get a reference to a blob
                BlobContainerClient containerClientOrig = blobServiceClient.GetBlobContainerClient("fau-iot");
                BlobClient blobClientOrig = containerClientOrig.GetBlobClient(fileNameOrig);

                // Open a stream and upload its data
                using (MemoryStream stream = new MemoryStream(copyOrigBytes))
                {
                    blobClientOrig.Upload(stream, true);
                }

                using (MemoryStream ms = new MemoryStream(audioChunk))
                {
                    using (var rs = new RawSourceWaveStream(ms, new WaveFormat(16000, 16, 1)))
                    {
                        var outputFormat = new WaveFormat(16000, 16, 1);
                        var inputProvider = new RawSourceWaveStream(ms, rs.WaveFormat);
                        var resampler = new WdlResamplingSampleProvider(inputProvider.ToSampleProvider(), outputFormat.SampleRate);
                        using (MemoryStream wms = new MemoryStream())
                        {
                            WaveFileWriter.WriteWavFileToStream(wms, resampler.ToWaveProvider16());

                            var reader = new BinaryReader(wms);
                            resampledBytes = new byte[wms.Length];
                            resampledBytes = reader.ReadBytes((int)wms.Length);

                            if (writeToFile)
                            {
                                var copyBytes = new byte[resampledBytes.Length - 1];
                                Array.Copy(resampledBytes, copyBytes, resampledBytes.Length - 1);
                                resampledBytes = copyBytes;

                                // Get a reference to a blob
                                BlobContainerClient containerClient = blobServiceClient.GetBlobContainerClient("fau-iot");
                                BlobClient blobClient = containerClient.GetBlobClient(fileName);

                                // Open a stream and upload its data
                                using (MemoryStream stream = new MemoryStream(copyBytes))
                                {
                                    blobClient.Upload(stream, true);
                                }
                            }
                        }
                        _logger.LogInformation($"Resampled Audio File: {fileName}");
                    }
                }
            }
            catch (Exception ex)
            {
                _logger.LogError($"Resample Audio Stream Error: {ex.Message}");

            }
            return resampledBytes;
        }


        /// <summary>
        /// Performs the audio stream to text transcription using Speech API
        /// Takes the AudioBuffer byte array and pushes the audio stream
        /// to the Speech API using a conversationTranscriber class.
        /// The conversationTranscriber class publishes events to listen
        /// for that allows the code to determine when or if a transcription
        /// is created for the audio buffer that was sent in.
        /// </summary>
        /// <param name="audioBuffer">Byte array of pure audio 16000, 8bits 1 channel</param>
        /// <param name="audioBufferLength">length of the byte array</param>
        /// <returns>N/A</returns>
        async Task RecognitionWithPushAudioStreamAsync(byte[] audioBuffer, int audioBufferLength)
        {

            try
            {

                stopRecognition =  new TaskCompletionSource<int>(TaskCreationOptions.RunContinuationsAsynchronously);

                // Create a push stream
                using (var pushStream = AudioInputStream.CreatePushStream())
                {
                    //                  _logger.LogInformation("StreamPush Created");
                    using (var audioInput = AudioConfig.FromStreamInput(pushStream))
                    {
                        config.SpeechRecognitionLanguage = "en-US";
                        
                        //                    _logger.LogInformation("audioInput Created");
                        // Creates a speech recognizer using audio stream input.
                        using (var recognizer = new SpeechRecognizer(config, audioInput))
                        {
                            
                            //                      _logger.LogInformation("recognizer Created");
                            var phraseList = PhraseListGrammar.FromRecognizer(recognizer);

                            //phraseList.AddPhrase("Turn On");
                            //phraseList.AddPhrase("Turn Off");
                            //phraseList.AddPhrase("Lock");
                            //phraseList.AddPhrase("Unlock");
                            //phraseList.AddPhrase("Turn Up");
                            //phraseList.AddPhrase("Turn Down");
                            //phraseList.AddPhrase("Toggle");
                            //phraseList.AddPhrase("Toggle Voice");

                            // Subscribes to events.
                            recognizer.Recognizing += (s, e) =>
                            {
                                _logger.LogInformation($"RECOGNIZING: Text={e.Result.Text}");
                            };

                            recognizer.Recognized += (s, e) =>
                            {
                                if (e.Result.Reason == ResultReason.RecognizedSpeech)
                                {
                                    //var bestResults = e.Result.Best();
                                    //if (bestResults != null)
                                    //{
                                    //    foreach (var best in bestResults)
                                    //    {
                                    //        if (best.Confidence >= confidenceFactor)
                                    //        {
                                    //            recognizedText.Append(best.Text);
                                    //        }
                                    //    }
                                    //}   
                                    //else
                                    //{
                                    recognizedText.Append(e.Result.Text);
                                    //}
                                    _logger.LogInformation($"{e.Result.Text} ");

                                }
                                else if (e.Result.Reason == ResultReason.NoMatch)
                                {
                                    _logger.LogInformation($"NOMATCH: Speech could not be recognized.");
                                }
                            };

                            recognizer.Canceled += (s, e) =>
                            {
                                _logger.LogInformation($"CANCELED: Reason={e.Reason}");

                                if (e.Reason == CancellationReason.Error)
                                {
                                    _logger.LogInformation($"\nCANCELED: ErrorCode={e.ErrorCode} - Details={e.ErrorDetails}");
                                    //_logger.LogInformation($"CANCELED: ErrorDetails={e.ErrorDetails}");
                                    //_logger.LogInformation($"CANCELED: Did you update the subscription info?");
                                }

                                stopRecognition.TrySetResult(0);
                            };

                            recognizer.SessionStarted += (s, e) =>
                            {
                                _logger.LogInformation("\nSession started event.");
                                // if session is still going wait until stopped to get remaining audio chunks
                            };

                            recognizer.SessionStopped += (s, e) =>
                            {
                                _logger.LogInformation("\nSession stopped event.");
                                //_logger.LogInformation("\nStop recognition.");
                                // stopRecognition.TrySetResult(0);
                                //_logger.LogInformation("\nStop Reason: " + e);
                                stopRecognition.TrySetResult(0);
                            };

                            // open and read the wave file and push the buffers into the recognizer
                            using (BinaryAudioStreamReader reader = new BinaryAudioStreamReader(new MemoryStream(audioBuffer, 0, audioBufferLength)))
                            {
                                // _logger.LogInformation("AudioStream Reader Created");
                                byte[] buffer = new byte[audioBufferLength ];
                                while (true)
                                {
                                    var readSamples = reader.Read(buffer, (uint)buffer.Length);
                                    if (readSamples == 0)
                                    {
                                        break;
                                    }
                                    pushStream.Write(buffer, readSamples);
                                }
                            }
                            pushStream.Close();


                            // Starts single recognition. It will stop after the first utterance is recognized.
                            _logger.LogInformation("Starting Transcribing.");
                            //var result = await recognizer.RecognizeOnceAsync();
                            
                            await recognizer.StartContinuousRecognitionAsync().ConfigureAwait(false);

                            await Task.WhenAll(stopRecognition.Task).ConfigureAwait(false);

                            await recognizer.StopContinuousRecognitionAsync().ConfigureAwait(false);

                            
                            // Check the result
                            //if (result.Reason == ResultReason.RecognizedSpeech)
                            //{                                
                            //    _logger.LogInformation($"We recognized: {result.Text}");
                            //    recognizedText.Append(result.Text);
                            //}
                            //else if (result.Reason == ResultReason.NoMatch)
                            //{
                            //    NoMatchDetails details = NoMatchDetails.FromResult(result);                                
                            //    _logger.LogInformation($"NOMATCH: Speech could not be recognized. Details: {details}");
                                
                            //}
                            //else if (result.Reason == ResultReason.Canceled)
                            //{
                            //    var cancellation = CancellationDetails.FromResult(result);
                            //    _logger.LogInformation($"CANCELED: Reason={cancellation.Reason}");

                            //    if (cancellation.Reason == CancellationReason.Error)
                            //    {
                            //        _logger.LogInformation($"CANCELED: ErrorCode={cancellation.ErrorCode}");
                            //        _logger.LogInformation($"CANCELED: ErrorDetails={cancellation.ErrorDetails}");

                            //    }
                            //}


                        }
                    }
                }

            }
            catch (Exception ex)
            {
                _logger.LogError(message: $"Exception on Transcription={ex.Message}");
            }
        }

    }


}

