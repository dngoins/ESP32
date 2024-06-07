using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Azure.Functions.Worker.Http;
using System.Net;

namespace azfunction
{
    public class negotiate
    {
        private readonly ILogger<negotiate> _logger;

        public negotiate(ILogger<negotiate> logger)
        {
            _logger = logger;
        }

        [Function("negotiate")]      
        public HttpResponseData Run([HttpTrigger(AuthorizationLevel.Anonymous, "get", "post")] HttpRequestData req,
            [WebPubSubConnectionInput(Hub = "notification", UserId ="iot")] WebPubSubConnection connectionInfo)
        {            
            var response = req.CreateResponse(HttpStatusCode.OK);
            response.WriteAsJsonAsync(connectionInfo);
            return response;
        }
    }
}
