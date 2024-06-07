using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Azure.Functions.Worker.Http;

namespace azfunction
{
    public class audioSample
    {
        private readonly ILogger<audioSample> _logger;

        public audioSample(ILogger<audioSample> logger)
        {
            _logger = logger;
        }


        [Function("audioSample")]
        public HttpResponseData Run([HttpTrigger(AuthorizationLevel.Anonymous, "get", "post")] HttpRequestData req, FunctionContext context)
        {
            var path = Path.Combine(context.FunctionDefinition.PathToAssembly, "../media/ToggleVoice.wav");
            _logger.LogInformation($"ToggleVoice Wave: {path}.");

            var response = req.CreateResponse();
            
            response.Headers.Add("Content-Type", "audio/wav");
            response.WriteBytesAsync(File.ReadAllBytes(path));
            return response;
        }
    }
}
