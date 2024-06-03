using System;
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;

namespace azfunction
{
    public class notification
    {
        private readonly ILogger _logger;

        public notification(ILoggerFactory loggerFactory)
        {
            _logger = loggerFactory.CreateLogger<notification>();
        }

        [Function("notification")]
        [WebPubSubOutput(Hub = "notification")]
        public SendToAllAction Run([TimerTrigger("*/10 * * * * *")] MyTimer myTimer)
        {
            return new SendToAllAction
            {
                Data = BinaryData.FromString($"[DateTime: {DateTime.Now}] Temperature: {GetValue(23, 1)}{'\xB0'}C, Humidity: {GetValue(40, 2)}%"),
                DataType = WebPubSubDataType.Text
            };
        }

       
        private static string GetValue(double baseNum, double floatNum)
        {
            var rng = new Random();
            var value = baseNum + floatNum * 2 * (rng.NextDouble() - 0.5);
            return value.ToString("0.000");
        }
    }
}
