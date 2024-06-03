using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;

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
        [WebPubSubOutput(Hub = "notification")]
        public SendToAllAction Run(
            [WebPubSubTrigger("notification", WebPubSubEventType.User, "audioSample")] UserEventRequest request)
        {
            return new SendToAllAction
            {
                Data = BinaryData.FromString($"[{request.ConnectionContext.UserId}] {request.Data.ToString()}"),
                DataType = request.DataType
            };
        }
    }
}
