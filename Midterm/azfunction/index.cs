using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Azure.Functions.Worker.Http;

namespace azfunction
{
    public class index
    {
        private readonly ILogger<index> _logger;

        public index(ILogger<index> logger)
        {
            _logger = logger;
        }

        [Function("index")]
        public HttpResponseData Run([HttpTrigger(AuthorizationLevel.Anonymous, "get", "post")] HttpRequestData req, FunctionContext context)
        {
            var path = Path.Combine(context.FunctionDefinition.PathToAssembly, "..\\index.html");
            _logger.LogInformation($"index.html path: {path}.");

            var response = req.CreateResponse();
            response.Headers.Add("Content-Type", "text/html");
            response.WriteStringAsync(File.ReadAllText(path));
            
            return response;
        }
    }
}
