#include "responses_client.hpp"

#include <cstdlib>
#include <memory>
#include <stdexcept>

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace vln_memory
{
namespace
{
/** @brief 限制接收的 JSON 大小，异常不能穿过 libcurl 的 C 回调。 */
std::size_t appendResponse(char * bytes, std::size_t size, std::size_t count, void * userdata) noexcept
{
  auto & response = *static_cast<std::string *>(userdata);
  const auto length = size * count;
  constexpr std::size_t max_bytes = 8 * 1024 * 1024;
  if (length > max_bytes - response.size()) {return 0;}
  try {response.append(bytes, length);} catch (...) {return 0;}
  return length;
}

/** @brief 为整个进程初始化 HTTP 库，并在退出时释放。 */
struct CurlRuntime
{
  CURLcode code{curl_global_init(CURL_GLOBAL_DEFAULT)};
  ~CurlRuntime() {if (code == CURLE_OK) {curl_global_cleanup();}}
};

/** @brief 支持 YAML 写 API 基地址或完整 /responses 路径。 */
std::string responseUrl(std::string url)
{
  while (!url.empty() && url.back() == '/') {url.pop_back();}
  constexpr std::string_view suffix = "/responses";
  if (url.size() < suffix.size() || url.compare(url.size() - suffix.size(), suffix.size(), suffix) != 0) {
    url += suffix;
  }
  return url;
}

/** @brief 发请求前读取本地密钥；未配置文件时兼容旧环境变量。 */
std::string modelApiKey(const ModelBuildOptions & options)
{
  if (!options.api_key_file.empty()) {
    try {
      const auto key = YAML::LoadFile(options.api_key_file.string())["api_key"];
      if (key.IsScalar()) {return key.as<std::string>();}
    } catch (const YAML::Exception &) {
      // 只报告配置路径，避免把文件内容带入错误输出。
    }
    throw std::runtime_error(
            "Cannot read model key from " + options.api_key_file.string() +
            "; set api_key in that YAML (use an empty string for no authentication)");
  }
  if (options.api_key_env.empty()) {return {};}
  const auto * key = std::getenv(options.api_key_env.c_str());
  if (!key || !*key) {
    throw std::runtime_error("Model key environment variable is unset: " + options.api_key_env);
  }
  return key;
}
}  // namespace

nlohmann::json requestResponse(const ModelBuildOptions & options, const nlohmann::json & request)
{
  static const CurlRuntime runtime;
  if (runtime.code != CURLE_OK) {throw std::runtime_error("Cannot initialize libcurl");}
  const std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(curl_easy_init(), curl_easy_cleanup);
  if (!curl) {throw std::runtime_error("Cannot open HTTP request");}
  const auto url = responseUrl(options.base_url);
  const auto key = modelApiKey(options);
  if (key.find_first_of("\r\n") != std::string::npos) {
    throw std::invalid_argument("Model key contains a line break");
  }
  const auto authorization = key.empty() ? std::string{} : "Authorization: Bearer " + key;
  auto * list = curl_slist_append(nullptr, "Content-Type: application/json");
  if (!list) {throw std::runtime_error("Cannot allocate HTTP headers");}
  std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(list, curl_slist_free_all);
  if (!authorization.empty() && !curl_slist_append(list, authorization.c_str())) {
    throw std::runtime_error("Cannot allocate HTTP authorization header");
  }

  const auto body = request.dump();
  std::string response;
  curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, body.data());
  curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, options.timeout_s);
  curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, appendResponse);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
  const auto status = curl_easy_perform(curl.get());
  if (status != CURLE_OK) {
    throw std::runtime_error(std::string("Responses request failed: ") + curl_easy_strerror(status));
  }
  long http_status = 0;
  curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &http_status);
  if (http_status < 200 || http_status >= 300) {
    throw std::runtime_error("Responses HTTP status " + std::to_string(http_status));
  }
  auto parsed = nlohmann::json::parse(response, nullptr, false);
  if (!parsed.is_object()) {throw std::runtime_error("Responses returned invalid JSON");}
  return parsed;
}
}  // namespace vln_memory
