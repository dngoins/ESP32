using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.SignalR;

namespace azfunction
{
    public class message
    {
        private readonly ILogger<message> _logger;

        public message(ILogger<message> logger)
        {
            _logger = logger;
        }

        [Function("message")]
        [WebPubSubOutput(Hub = "notification")]
        public SendToAllAction Run(
            [WebPubSubTrigger("notification", WebPubSubEventType.User, "message")] UserEventRequest request)
        {
            _logger.LogInformation($"Request from: {request.ConnectionContext.UserId}");
            _logger.LogInformation($"Request message data: {request.Data}");
            _logger.LogInformation($"Request message dataType: {request.DataType}");
            return new SendToAllAction
            {
                Data = BinaryData.FromString($"[{request.ConnectionContext.UserId}] {request.Data.ToString()}"),
                DataType = request.DataType
            };
        }

    }
}
